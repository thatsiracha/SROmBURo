"""
SrOmBURo balance controller — torque-mode PD
=============================================
PD structure mirrors control2.py; output is motor torque [Nm] via setTorque
instead of motor velocity [rad/s] via setVelocity.

Control law (sign convention identical to control2.py):
  tau_wheel  = −(KT11·pitch + KT12·pitchdot + KT13·phi1dot + KT14·phi_w + KTI_PITCH·pitch_int)
  tau_roller = +(KT21·roll  + KT22·rolldot  + KTI_ROLL·roll_int) − KT23·phi2dot − KT24·phi_r

  pitch > 0 → tau_wheel  < 0  (wheel brakes/reverses — same as control2.py)
  roll  > 0 → tau_roller > 0  (roller drives    — same as control2.py)

phi_w = PHI_POS_SIGN_W × (pos_w − zero)   encoder position [motor rad]
phi_r = PHI_POS_SIGN_R × (pos_r − zero)   encoder position [motor rad]

No N_ROLLER multiply — gains are tuned for motor torque directly.
No differential command — setTorque(tau_w, tau_r) sends independent torques.
Soft-start: output ramps 0 → 1 over SOFTSTART_STEPS (~0.5 s).

Tune order:
  1. KT11, KT12 only (set KT13=KT14=KTI_PITCH=0) — pitch PD
  2. KT21, KT22 only (set KT23=KT24=KTI_ROLL=0)  — roll PD
  3. Add KT13/KT23 (wheel-velocity damping)
  4. Add KT14/KT24 (drift/position correction via encoder)
  5. Add KTI_PITCH/KTI_ROLL (station-keeping integral)
"""

import math
import struct
import sys
import threading
import time

import numpy as np
import serial

sys.path.insert(0, "/home/omburo/Documents/SROmBURo/Jun")
from Omburo_JUN import Omburo

# ── Hardware ──────────────────────────────────────────────────────────────────
IMU_PORT    = "/dev/ttyACM0"
IMU_BAUD    = 115200
IMU_RATE_HZ = 200

# ── Physical ──────────────────────────────────────────────────────────────────
R        = 0.101
r        = 0.0142
N_ROLLER = 4.0

# ── IMU axis mapping (preserved from control2.py) ─────────────────────────────
ROLL_EU_IDX  = 0;  ROLL_EU_SIGN  =  1.0
PITCH_EU_IDX = 1;  PITCH_EU_SIGN = -1.0
ROLLDOT_IDX  = 0;  ROLLDOT_SIGN  =  1.0
PITCHDOT_IDX = 1;  PITCHDOT_SIGN = -1.0

# ── Safety & timing ───────────────────────────────────────────────────────────
FALL_DEG = 30.0
CTRL_HZ  = 150
CTRL_DT  = 1.0 / CTRL_HZ

KT       = 0.35           # Nm/A — must match Omburo.py
I_LIM_A  = 6.0            # A
TAU_MAX  = KT * I_LIM_A   # 2.1 Nm hard clamp

SOFTSTART_STEPS = int(0.5 * CTRL_HZ)   # 75 steps — ramp output 0→1 over ~0.5 s

# Encoder position sign convention (see LQR_NOTES.md):
#   readback() negates vel_w but NOT pos_w → negate pos_w to match vel sign.
#   If robot jolts/falls immediately on enable, flip PHI_POS_SIGN_W to +1.0.
PHI_POS_SIGN_W = -1.0
PHI_POS_SIGN_R =  1.0

# ── PD gains ──────────────────────────────────────────────────────────────────
# Units: angle gains [Nm/rad], rate gains [Nm·s/rad], integral gain [Nm/(rad·s)]
# LQR optimal (compute_lqr.py at nominal params) shown as reference.
# Start with KT13=KT14=KTI_PITCH=KT23=KT24=KTI_ROLL=0 until pitch/roll stable.

# Wheel (longitudinal — pitch axis)
KT11 = 60.0   # pitch angle        [Nm/rad]    LQR ref: 69.6
KT12 = 15.0   # pitchdot           [Nm·s/rad]  LQR ref: 19.7
KT13 =  0.0   # phi1dot wheel vel  [Nm·s/rad]  LQR ref:  1.4  (enable after stable)
KT14 =  0.0   # phi_w wheel pos    [Nm/rad]    LQR ref:  1.0  (enable after stable)
KTI_PITCH = 0.0   # pitch integral [Nm/(rad·s)]               (enable last)

# Roller (lateral — roll axis)
KT21 = 50.0   # roll angle         [Nm/rad]    LQR ref: 59.1
KT22 = 12.0   # rolldot            [Nm·s/rad]  LQR ref: 17.7
KT23 =  0.0   # phi2dot roller vel [Nm·s/rad]  LQR ref:  1.4  (enable after stable)
KT24 =  0.0   # phi_r roller pos   [Nm/rad]    LQR ref:  1.0  (enable after stable)
KTI_ROLL  = 0.0   # roll integral  [Nm/(rad·s)]               (enable last)

INT_TAU_LIM = 0.8   # max integral torque contribution [Nm] — anti-windup

# ── State filters (same as control2.py) ──────────────────────────────────────
EMA_ALPHA_ANG  = 0.2
EMA_ALPHA_RATE = 0.35
EMA_ALPHA_PHI  = 0.6


# ─── MIP (MicroStrain Inertial Protocol) helpers (preserved verbatim) ─────────
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
        self.robot = Omburo()

        self._pitch_int    = 0.0
        self._roll_int     = 0.0
        self._roll_offset  = 0.0
        self._pitch_offset = 0.0

        self._phi_w_zero   = 0.0
        self._phi_r_zero   = 0.0

        self._softstart_cnt = 0

        self._last_pos_w = 0.0;  self._last_vel_w = 0.0
        self._last_pos_r = 0.0;  self._last_vel_r = 0.0

        self._roll_filt     = 0.0
        self._pitch_filt    = 0.0
        self._rolldot_filt  = 0.0
        self._pitchdot_filt = 0.0
        self._phi1dot_filt  = 0.0
        self._phi2dot_filt  = 0.0

        self._dbg_count = 0

    def _calibrate(self, n=50):
        roll_sum = pitch_sum = 0.0
        print(f"Calibrating IMU ({n} samples) — hold robot still...")
        for _ in range(n):
            self.imu.updated.wait(); self.imu.updated.clear()
            euler, _ = self.imu.get()
            roll_sum  += ROLL_EU_SIGN  * math.radians(euler[ROLL_EU_IDX])
            pitch_sum += PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX])
        self._roll_offset  = roll_sum  / n
        self._pitch_offset = pitch_sum / n
        self._pitch_int = 0.0
        self._roll_int  = 0.0
        print(f"  roll offset={math.degrees(self._roll_offset):.2f}°  "
              f"pitch offset={math.degrees(self._pitch_offset):.2f}°")

        try:
            pos_w, _, pos_r, _ = self.robot.readback()
            self._phi_w_zero = pos_w
            self._phi_r_zero = pos_r
            print(f"  encoder zeros: wheel={pos_w:.4f} rad  roller={pos_r:.4f} rad")
        except Exception as exc:
            print(f"  [WARN] encoder zero read failed ({exc}); using 0.0")
            self._phi_w_zero = 0.0
            self._phi_r_zero = 0.0

    def run(self):
        t = threading.Thread(target=_imu_thread, args=(self.imu,), daemon=True)
        t.start()

        print(f"[ctrl] torque PD  TAU_MAX={TAU_MAX:.2f} Nm  soft-start={SOFTSTART_STEPS} steps")
        print(f"[ctrl] PITCH: KT11={KT11}  KT12={KT12}  KT13={KT13}  KT14={KT14}  KTI={KTI_PITCH}")
        print(f"[ctrl] ROLL:  KT21={KT21}  KT22={KT22}  KT23={KT23}  KT24={KT24}  KTI={KTI_ROLL}")

        print("Waiting for IMU data...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond — aborting")
            self.robot.close()
            return

        self._calibrate()
        self.robot.setTorqueMode()
        self.robot.toggleTorque(1)
        self._softstart_cnt = 0
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
            self.robot.setTorque(0.0, 0.0)
            time.sleep(0.02)
            self.robot.toggleTorque(0)
            self.robot.setTorqueMode()
            self.robot.close()

    def _step(self):
        self._dbg_count += 1
        euler, gyro = self.imu.get()

        roll     = ROLL_EU_SIGN  * math.radians(euler[ROLL_EU_IDX])  - self._roll_offset
        pitch    = PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX]) - self._pitch_offset
        rolldot  = ROLLDOT_SIGN  * gyro[ROLLDOT_IDX]
        pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]

        # Fall-stop
        if abs(roll) > math.radians(FALL_DEG) or abs(pitch) > math.radians(FALL_DEG):
            self.robot.setTorque(0.0, 0.0)
            return

        # Encoder readback
        try:
            pos_w, vel_w, pos_r, vel_r = self.robot.readback()
            self._last_pos_w = pos_w;  self._last_vel_w = vel_w
            self._last_pos_r = pos_r;  self._last_vel_r = vel_r
        except Exception:
            pos_w = self._last_pos_w;  vel_w = self._last_vel_w
            pos_r = self._last_pos_r;  vel_r = self._last_vel_r

        phi1dot = vel_w   # wheel motor velocity [rad/s] (sign already corrected in readback)
        phi2dot = vel_r   # roller motor velocity [rad/s]

        # EMA low-pass filters
        self._roll_filt     = EMA_ALPHA_ANG  * self._roll_filt     + (1 - EMA_ALPHA_ANG)  * roll
        self._pitch_filt    = EMA_ALPHA_ANG  * self._pitch_filt    + (1 - EMA_ALPHA_ANG)  * pitch
        self._rolldot_filt  = EMA_ALPHA_RATE * self._rolldot_filt  + (1 - EMA_ALPHA_RATE) * rolldot
        self._pitchdot_filt = EMA_ALPHA_RATE * self._pitchdot_filt + (1 - EMA_ALPHA_RATE) * pitchdot
        self._phi1dot_filt  = EMA_ALPHA_PHI  * self._phi1dot_filt  + (1 - EMA_ALPHA_PHI)  * phi1dot
        self._phi2dot_filt  = EMA_ALPHA_PHI  * self._phi2dot_filt  + (1 - EMA_ALPHA_PHI)  * phi2dot

        roll     = self._roll_filt;     pitch    = self._pitch_filt
        rolldot  = self._rolldot_filt;  pitchdot = self._pitchdot_filt
        phi1dot  = self._phi1dot_filt;  phi2dot  = self._phi2dot_filt

        # Encoder position relative to calibration zero [motor rad]
        phi_w = PHI_POS_SIGN_W * (pos_w - self._phi_w_zero)
        phi_r = PHI_POS_SIGN_R * (pos_r - self._phi_r_zero)

        # Soft-start ramp
        ramp = min(1.0, self._softstart_cnt / SOFTSTART_STEPS)
        if self._softstart_cnt < SOFTSTART_STEPS:
            self._softstart_cnt += 1

        # Angle integral — anti-windup
        int_max = INT_TAU_LIM / max(KTI_PITCH, 1e-9)
        self._pitch_int = float(np.clip(
            self._pitch_int + pitch * CTRL_DT, -int_max, int_max))
        int_max_r = INT_TAU_LIM / max(KTI_ROLL, 1e-9)
        self._roll_int = float(np.clip(
            self._roll_int + roll * CTRL_DT, -int_max_r, int_max_r))

        # ── Torque PD law ────────────────────────────────────────────────────
        tau_wheel = -(KT11 * pitch + KT12 * pitchdot
                      + KT13 * phi1dot + KT14 * phi_w
                      + KTI_PITCH * self._pitch_int)

        tau_roller = (KT21 * roll + KT22 * rolldot
                      + KTI_ROLL * self._roll_int
                      - KT23 * phi2dot - KT24 * phi_r)

        tau_wheel  = float(np.clip(tau_wheel,  -TAU_MAX, TAU_MAX)) * ramp
        tau_roller = float(np.clip(tau_roller, -TAU_MAX, TAU_MAX)) * ramp

        self.robot.setTorque(tau_wheel, tau_roller)

        if self._dbg_count % 20 == 0:
            print(
                f"\n"
                f"  angle  : roll={math.degrees(roll):+6.2f}°  pitch={math.degrees(pitch):+6.2f}°\n"
                f"  rate   : rdot={rolldot:+5.2f}  pdot={pitchdot:+5.2f}  rad/s\n"
                f"  phi    : φ_w={phi_w:+7.3f} rad  φ_r={phi_r:+7.3f} rad"
                f"  (phi1dot={phi1dot:+5.2f}  phi2dot={phi2dot:+5.2f} rad/s)\n"
                f"  tau    : wheel={tau_wheel:+6.3f} Nm  roller={tau_roller:+6.3f} Nm"
                f"  ramp={ramp:.2f}",
                flush=True
            )


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = Controller()
    ctrl.run()
