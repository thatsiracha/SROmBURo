"""
SrOmBURo velocity-loop balancing controller for Raspberry Pi

Mirrors OmburoRev2 (Arduino/Dynamixel) control structure, adapted for
BEAR actuators (pybear, rad/s output) and SrOmBURo physical parameters.

Per-axis PD control law:
    v_cmd = Kp·θ + Kd·θ̇ + Kφ̇·φ̇ + Kφ·φ_pos   [rad/s]

Motor output, matching the SrOmBURo BEAR wiring:
    motor id2 (big wheel) = pitch_cmd — corrects pitch (longitudinal)
    motor id1 (roller)    = roll_cmd  — corrects roll (lateral)

OmburoVel.setVelocity() takes arguments in hardware order:
    setVelocity(vel_wheel, vel_roller)   # vel_wheel → id2, vel_roller → id1

Gain starting values scaled from OmburoRev2:
    Dynamixel unit → rad/s  ×0.01194  (0.114 RPM/unit × 2π/60)

Reference: Shen & Hong, "OmBURo: A Novel Unicycle Robot with Active
           Omnidirectional Wheel," arXiv:2001.07856, 2020.
"""

import math
import struct
import sys
import threading
import time

import numpy as np
import serial
from pybear import Manager

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from OmburoVel import OmburoVel

# ── Hardware ──────────────────────────────────────────────────────────────────
IMU_PORT    = "/dev/ttyACM0"
IMU_BAUD    = 115200
IMU_RATE_HZ = 200          # must divide 500 evenly

ID_WHEEL    = 2            # BEAR motor id2 (big wheel): pitch axis → receives pitch_cmd
ID_ROLLER   = 1            # BEAR motor id1 (roller):     roll axis  → receives roll_cmd

# ── Physical parameters (SrOmBURo) ───────────────────────────────────────────
R        = 0.101    # wheel radius [m]
r        = 0.0142   # roller radius [m]
N_ROLLER = 4.0      # roller gear coupling ratio (φ̇2 = N·(φ̇_w + φ̇_r))

# ── IMU axis mapping ──────────────────────────────────────────────────────────
# Matches OmburoRev2: euler[0]=roll (lateral θ1), euler[1]=pitch (longitudinal θ2)
# Adjust signs for physical IMU mounting orientation.
ROLL_EU_IDX  = 0;  ROLL_EU_SIGN  =  1.0   # roll  → θ1 (lateral)
PITCH_EU_IDX = 1;  PITCH_EU_SIGN = -1.0  # pitch → θ2 (longitudinal)  [inverted: hw test confirmed]
ROLLDOT_IDX  = 0;  ROLLDOT_SIGN  =  1.0  # ωx    → θ̇1
PITCHDOT_IDX = 1;  PITCHDOT_SIGN = -1.0  # ωy    → θ̇2                [inverted: same axis as pitch]

# ── Safety & timing ───────────────────────────────────────────────────────────
FALL_DEG = 40.0    # fall-stop threshold [deg]
VEL_MAX  = 50.0    # motor velocity command saturation [rad/s]
CTRL_HZ  = 500
CTRL_DT  = 1.0 / CTRL_HZ

# ── Control gains ─────────────────────────────────────────────────────────────
# Axis 1: roll correction, matching OmburoRev2 f1_goal
K11 = 300.0   # roll angle gain
K12 = 15.0   # rolldot gain
K13 =  0.0   # φ̇ gain
K14 =  0.0  # φ position gain (roll) — keep tiny
H1  =  0.0   # feedforward — disabled

# Axis 2: pitch correction, matching OmburoRev2 f2_goal
K21 = 30.0   # pitch angle gain
K22 =  15.0   # pitchdot gain
K23 =  0.0   # φ̇ gain
K24 =  0.0  # φ position gain (pitch) — keep tiny
H2  =  0.0   # feedforward — disabled

# ── State measurement filter ──────────────────────────────────────────────────
EMA_ALPHA_ANG  = 0.2    # angle  filter (~8 Hz cutoff @ 150 Hz)
EMA_ALPHA_RATE = 0.35   # rate   filter (~5 Hz cutoff @ 150 Hz)
EMA_ALPHA_PHI  = 0.6    # φ̇ filter  (~3 Hz cutoff @ 150 Hz) — encoder noisier than gyro


# ─── MIP (MicroStrain Inertial Protocol) helpers ──────────────────────────────
MIP_SYNC1  = 0x75;  MIP_SYNC2  = 0x65
DESC_BASE  = 0x01;  DESC_3DM   = 0x0C;  DESC_IMU  = 0x80
FIELD_GYRO = 0x05;  FIELD_EULER = 0x0C
MIP_BASE_RATE = 500

def _fletcher(data: bytes):
    b1 = b2 = 0
    for b in data:
        b1 = (b1 + b) & 0xFF
        b2 = (b2 + b1) & 0xFF
    return b1, b2

def _build_field(desc, data=b""):
    return bytes([len(data) + 2, desc]) + data

def _build_packet(desc_set, fields):
    payload = b"".join(fields)
    header  = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
    body    = header + payload
    cs1, cs2 = _fletcher(body)
    return body + bytes([cs1, cs2])

def _cmd_ping():
    return _build_packet(DESC_BASE, [_build_field(0x01)])

def _cmd_imu_format(rate_hz, field_list):
    dec  = max(1, MIP_BASE_RATE // rate_hz)
    data = bytes([0x01, len(field_list)])
    for f in field_list:
        data += bytes([f, dec >> 8, dec & 0xFF])
    return _build_packet(DESC_3DM, [_build_field(0x08, data)])

def _cmd_imu_stream(enable: bool):
    data = bytes([0x01, 0x01, 0x01 if enable else 0x00])
    return _build_packet(DESC_3DM, [_build_field(0x11, data)])

def _read_mip_packet(ser, timeout=2.0):
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while True:
            idx = buf.find(bytes([MIP_SYNC1, MIP_SYNC2]))
            if idx == -1:
                buf = buf[-1:]; break
            buf = buf[idx:]
            if len(buf) < 4: break
            plen  = buf[3]
            total = 4 + plen + 2
            if len(buf) < total: break
            pkt = buf[:total]
            if _fletcher(pkt[:-2]) != (pkt[-2], pkt[-1]):
                buf = buf[1:]; continue
            buf = buf[total:]
            return pkt[2], pkt[4: 4 + plen]
    return None

def _imu_ack(ser, pkt, cmd_desc, retries=3):
    for _ in range(retries):
        ser.reset_input_buffer(); ser.write(pkt)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            r = _read_mip_packet(ser, timeout=deadline - time.time())
            if r is None: break
            ds, pl = r
            if ds == DESC_IMU: continue
            if len(pl) >= 4 and pl[1] == 0xF1:
                if pl[2] == cmd_desc and pl[3] == 0x00:
                    return True
                break
    return False

def _parse_imu_payload(payload):
    euler = gyro = None
    i = 0
    while i + 1 < len(payload):
        flen = payload[i]; fdesc = payload[i + 1]
        fdata = payload[i + 2: i + flen]
        if fdesc == FIELD_EULER and len(fdata) >= 12:
            roll, pitch, yaw = struct.unpack(">fff", fdata[:12])
            euler = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        elif fdesc == FIELD_GYRO and len(fdata) >= 12:
            gyro = struct.unpack(">fff", fdata[:12])
        i += max(flen, 1)
    return euler, gyro


# ─── Thread-safe IMU state ────────────────────────────────────────────────────
class IMUState:
    def __init__(self):
        self._lock   = threading.Lock()
        self.euler   = (0.0, 0.0, 0.0)
        self.gyro    = (0.0, 0.0, 0.0)
        self.updated = threading.Event()

    def push(self, euler=None, gyro=None):
        with self._lock:
            if euler is not None: self.euler = euler
            if gyro  is not None: self.gyro  = gyro
        self.updated.set()

    def get(self):
        with self._lock:
            return self.euler, self.gyro


def _imu_thread(state: IMUState):
    try:
        with serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.05,
                           dsrdtr=False, rtscts=False) as ser:
            time.sleep(1.5); ser.reset_input_buffer()
            if not _imu_ack(ser, _cmd_ping(), 0x01):
                print("[IMU] no ping response — check connection"); return
            # disable stream first in case previous run left it active
            _imu_ack(ser, _cmd_imu_stream(False), 0x11)
            time.sleep(0.1); ser.reset_input_buffer()
            if not _imu_ack(ser, _cmd_imu_format(IMU_RATE_HZ, [FIELD_EULER, FIELD_GYRO]), 0x08):
                print("[IMU] format setup failed"); return
            if not _imu_ack(ser, _cmd_imu_stream(True), 0x11):
                print("[IMU] stream enable failed"); return
            print("[IMU] stream active")
            while True:
                r = _read_mip_packet(ser, timeout=0.5)
                if r is None: continue
                ds, pl = r
                if ds != DESC_IMU: continue
                euler, gyro = _parse_imu_payload(pl)
                state.push(euler=euler, gyro=gyro)
    except Exception as exc:
        print(f"[IMU] thread error: {exc}")


# ─── Main controller ──────────────────────────────────────────────────────────
class Controller:
    def __init__(self):
        self.imu   = IMUState()
        self.robot = OmburoVel()

        self._vref1 = 0.0   # lateral   velocity reference [m/s]
        self._vref2 = 0.0   # longitudinal velocity reference [m/s]

        self._roll_offset  = 0.0  # set by _calibrate()
        self._pitch_offset = 0.0

        self._last_vw = 0.0     # last velocity command (readback fallback)
        self._last_vr = 0.0

        self._phi_pitch_pos = 0.0  # pitch axis: motor id2 (big wheel)
        self._phi_roll_pos  = 0.0  # roll axis:  motor id1 (roller)

        self._roll_filt  = 0.0  # EMA-filtered states
        self._pitch_filt = 0.0
        self._rolldot_filt  = 0.0
        self._pitchdot_filt = 0.0
        self._phi_pitchdot_filt = 0.0
        self._phi_rolldot_filt  = 0.0

    def set_velocity_reference(self, vx_ms: float, vy_ms: float):
        """Set desired Cartesian velocity [m/s]. Call before or during run()."""
        self._vref1 = vx_ms   # lateral      → roller axis (id1, roll)
        self._vref2 = vy_ms   # longitudinal → wheel axis  (id2, pitch)

    def _calibrate(self, n=50):
        """Average roll/pitch over n samples to remove static IMU bias."""
        roll_sum = pitch_sum = 0.0
        print(f"Calibrating IMU ({n} samples) — hold robot still...")
        for _ in range(n):
            self.imu.updated.wait(); self.imu.updated.clear()
            euler, _ = self.imu.get()
            roll_sum  += ROLL_EU_SIGN  * math.radians(euler[ROLL_EU_IDX])
            pitch_sum += PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX])
        self._roll_offset  = roll_sum  / n
        self._pitch_offset = pitch_sum / n
        print(f"  roll offset={math.degrees(self._roll_offset):.2f}°  "
              f"pitch offset={math.degrees(self._pitch_offset):.2f}°")

    def run(self):
        t = threading.Thread(target=_imu_thread, args=(self.imu,), daemon=True)
        t.start()

        print("Waiting for IMU data...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond — aborting")
            self.robot.close()
            return

        self._calibrate()
        self.robot.setVelocityMode()
        self.robot.toggleTorque(1)
        print("Balancing active — Ctrl+C to stop\n")

        t_next = time.perf_counter()
        _hz_count = 0
        _hz_ts    = time.perf_counter()
        try:
            while True:
                now  = time.perf_counter()
                wait = t_next - now
                if wait > 0:
                    time.sleep(wait * 0.9)
                    while time.perf_counter() < t_next:
                        pass
                t_next += CTRL_DT
                self._step()

                _hz_count += 1
                if _hz_count >= 150:
                    elapsed = time.perf_counter() - _hz_ts
                    print(f"\r[ctrl] {_hz_count / elapsed:.1f} Hz", end="", flush=True)
                    _hz_count = 0
                    _hz_ts = time.perf_counter()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.robot.setVelocity(0.0, 0.0)
            self.robot.toggleTorque(0)
            self.robot.setTorqueMode()
            self.robot.close()

    # ── Control step (runs at CTRL_HZ) ────────────────────────────────────────
    def _step(self):
        self._dbg_count = getattr(self, '_dbg_count', 0) + 1
        euler, gyro = self.imu.get()

        # IMU → body angles and rates (subtract calibration offset)
        roll     = ROLL_EU_SIGN  * math.radians(euler[ROLL_EU_IDX])  - self._roll_offset
        pitch    = PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX]) - self._pitch_offset
        rolldot  = ROLLDOT_SIGN  * gyro[ROLLDOT_IDX]
        pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]

        # Fall-stop
        if abs(roll) > math.radians(FALL_DEG) or abs(pitch) > math.radians(FALL_DEG):
            self.robot.setVelocity(0.0, 0.0)
            return

        # Read encoder velocity (same thread as setVelocity → no lock contention)
        try:
            _, vel_w, _, vel_r = self.robot.readback()
        except Exception:
            vel_w, vel_r = self._last_vw, self._last_vr

        # Encoder → axis rates (matches BEAR wiring):
        #   id1 (roller)    → roll axis
        #   id2 (big wheel) → pitch axis
        vel_id2 = vel_w   # id2 big wheel
        vel_id1 = vel_r   # id1 roller
        phi_rolldot  = vel_id1
        phi_pitchdot = vel_id2

        # EMA low-pass filter
        a_ang  = EMA_ALPHA_ANG
        a_rate = EMA_ALPHA_RATE
        a_phi  = EMA_ALPHA_PHI
        self._roll_filt     = a_ang  * self._roll_filt     + (1 - a_ang)  * roll
        self._pitch_filt    = a_ang  * self._pitch_filt    + (1 - a_ang)  * pitch
        self._rolldot_filt  = a_rate * self._rolldot_filt  + (1 - a_rate) * rolldot
        self._pitchdot_filt = a_rate * self._pitchdot_filt + (1 - a_rate) * pitchdot
        self._phi_pitchdot_filt = a_phi * self._phi_pitchdot_filt + (1 - a_phi) * phi_pitchdot
        self._phi_rolldot_filt  = a_phi * self._phi_rolldot_filt  + (1 - a_phi) * phi_rolldot

        roll     = self._roll_filt;     pitch    = self._pitch_filt
        rolldot  = self._rolldot_filt;  pitchdot = self._pitchdot_filt
        phi_pitchdot = self._phi_pitchdot_filt
        phi_rolldot  = self._phi_rolldot_filt

        # Wheel position integral
        self._phi_pitch_pos += phi_pitchdot * CTRL_DT
        self._phi_roll_pos  += phi_rolldot  * CTRL_DT

        # Axis 1, roll: OmburoRev2 f1_goal (PD).
        roll_cmd = K11 * roll + K12 * rolldot + K13 * phi_rolldot + K14 * self._phi_roll_pos
        # Axis 2, pitch: OmburoRev2 f2_goal (PD). Roller coupling scale on pitch axis.
        pitch_cmd = -(K21 * pitch + K22 * pitchdot + K23 * phi_pitchdot + K24 * self._phi_pitch_pos) * N_ROLLER

        # BEAR wiring: id2 (big wheel) = pitch_cmd; id1 (roller) = roll_cmd.
        vel_wheel  = float(np.clip(pitch_cmd, -VEL_MAX, VEL_MAX))  # id2 → pitch
        vel_roller = float(np.clip(roll_cmd,  -VEL_MAX, VEL_MAX))  # id1 → roll
        self.robot.setVelocity(vel_wheel, vel_roller)
        self._last_vw = vel_wheel
        self._last_vr = vel_roller

        # Debug log at ~10 Hz
        if self._dbg_count % 20 == 0:
            print(
                f"\n"
                f"  angle : roll={math.degrees(roll):+6.2f}°  pitch={math.degrees(pitch):+6.2f}°\n"
                f"  rate  : rdot={rolldot:+5.2f}  pdot={pitchdot:+5.2f}  rad/s\n"
                f"  axis  : pitch_cmd={pitch_cmd:+6.2f}  roll_cmd={roll_cmd:+6.2f}  rad/s\n"
                f"  motor : id2/pitch={vel_wheel:+6.2f}  id1/roll={vel_roller:+6.2f}  rad/s",
                flush=True
            )


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = Controller()
    # ctrl.set_velocity_reference(vx_ms, vy_ms)   # optional motion command
    ctrl.run()
