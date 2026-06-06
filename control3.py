"""
SrOmBURo balance controller — LQR current mode / hand-tuned velocity mode
==========================================================================

CONTROL_MODE = "current"   u = -K·x (motor torque via set_goal_iq)
CONTROL_MODE = "velocity"  original hand-tuned PD law (rad/s via set_goal_velocity)

Switch with the CONTROL_MODE flag below.  The velocity path is 100% preserved
from control2.py so you can revert with one edit.

LQR state per axis:  x = [φ, θ, φ̇, θ̇]
  φ     = wheel CONTACT angle [rad], relative to calibration zero
            PITCH: φ_w = −(pos_wheel − zero)     ← negate: readback negates vel but not pos
            ROLL:  φ_r = N_ROLLER × (pos_roller − zero)
  θ     = body lean [rad]  (positive = lean backward, per PITCH/ROLL sign constants)
  u     = motor torque [Nm]  (K gives motor torque directly for both axes)

Sign note:  PITCH: PHI_POS_SIGN_W = −1.0 assumed.  If robot jolts on enable,
            flip this to +1.0.  Same for PHI_POS_SIGN_R.
"""

import math
import struct
import sys
import threading
import time

import numpy as np
import serial

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from Omburo import Omburo

# ── LQR gains — computed by compute_lqr.py, pasted here as constants ──────────
# Re-run `python3 compute_lqr.py` after changing physical params or Q/R weights,
# then paste the printed K_PITCH / K_ROLL lines below.
# K = [K_φ, K_θ, K_φ̇, K_θ̇]  (all Nm/rad or Nm·s/rad)
# K_PITCH all POSITIVE  → u_pitch = -(K@x) is NEGATIVE when states positive
#                          (matches control2.py: v_wheel = -(K11·pitch + …))
# K_ROLL  all NEGATIVE  → u_roll  = -(K@x) is POSITIVE when states positive
#                          (matches control2.py: v_roller = +(K21·roll + …)·N)
K_PITCH = np.array([0.100000, 22.104548, 0.198828, 4.982078])
K_ROLL  = np.array([-0.100000, -9.286382, -0.151012, -2.431171])

# ── Mode flag ─────────────────────────────────────────────────────────────────
CONTROL_MODE = "current"    # "current" | "velocity"

# ── Hardware ──────────────────────────────────────────────────────────────────
IMU_PORT    = "/dev/ttyACM0"
IMU_BAUD    = 115200
IMU_RATE_HZ = 200

ID_WHEEL    = 2
ID_ROLLER   = 1

# ── Physical ──────────────────────────────────────────────────────────────────
R        = 0.101
r        = 0.0142
N_ROLLER = 4.0

# ── IMU axis mapping (matches control.py/control2.py) ─────────────────────────
ROLL_EU_IDX  = 0;  ROLL_EU_SIGN  =  1.0
PITCH_EU_IDX = 1;  PITCH_EU_SIGN = -1.0
ROLLDOT_IDX  = 0;  ROLLDOT_SIGN  =  1.0
PITCHDOT_IDX = 1;  PITCHDOT_SIGN = -1.0

# ── Safety & timing ───────────────────────────────────────────────────────────
FALL_DEG = 30.0
VEL_MAX  = 15.0    # velocity mode saturation [rad/s]

# Torque constants (must match Omburo.py)
KT       = 0.35    # Nm/A
I_LIM_A  = 6.0     # A (set in Omburo.py; don't exceed)
I_TEST_A = 1.5     # A software clamp for early LQR tests; raise gradually after sign checks
TAU_MAX  = KT * min(I_LIM_A, I_TEST_A)


CTRL_HZ  = 150
CTRL_DT  = 1.0 / CTRL_HZ

# Soft-start: ramp output 0→1 over this many steps (~0.5 s)
SOFTSTART_STEPS = int(0.5 * CTRL_HZ)   # 75 steps

# Encoder position sign convention:
#   PITCH: readback negates velocity but not position → negate pos to match vel
#   ROLL:  readback does NOT negate velocity → pos sign matches vel
# Flip to +1.0 / -1.0 if robot jolts unexpectedly on enable (sign verification).
PHI_POS_SIGN_W = -1.0   # wheel-motor pos sign relative to vel_w convention
PHI_POS_SIGN_R =  1.0   # roller-motor pos sign relative to vel_r convention

# Keep encoder feedback off until torque/current signs are verified on hardware.
# If a brief tilt makes the motor keep running, phi/phidot feedback is usually
# using the wrong sign or the wrong contact-velocity model.
USE_ENCODER_FEEDBACK_CURRENT = False

# How to convert axis torques to physical motor current commands.
# "control_ref": match control.py/control2.py output mixing:
#                setVelocity(v_wheel, v_roller - v_wheel)
# "independent": send wheel=u_pitch, roller=u_roll directly.
TORQUE_MIX_MODE = "control_ref"

# ── Hand-tuned velocity-mode gains (preserved, kept for comparison) ────────────
# Wheel (longitudinal, pitch):
K11 = 85.0;  K12 = 11.0;  K13 = 0.3;  K14 = 0.03;  H1 = 0.0
# Roller (lateral, roll):
K21 = 70.0;  K22 =  2.0;  K23 = 0.3;  K24 = 0.03;  H2 = 0.0
# Angle integrals:
KI_PITCH = 2.0;  KI_ROLL = 1.5;  INT_VEL_LIM = 3.0

# ── State filters ─────────────────────────────────────────────────────────────
EMA_ALPHA_ANG  = 0.2
EMA_ALPHA_RATE = 0.35
EMA_ALPHA_PHI  = 0.6    # heavier — encoder velocity noisier than gyro


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


# ─── Thread-safe IMU state (preserved) ───────────────────────────────────────
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

        self._vref1 = 0.0
        self._vref2 = 0.0

        self._pitch_int = 0.0
        self._roll_int  = 0.0
        self._roll_offset  = 0.0
        self._pitch_offset = 0.0

        # Encoder position zeros (set in _calibrate)
        self._phi_w_zero = 0.0
        self._phi_r_zero = 0.0

        # Soft-start counter (reset in run() before loop)
        self._softstart_cnt = 0

        # Last readback values (fallback on comm error)
        self._last_pos_w = 0.0;  self._last_vel_w = 0.0
        self._last_pos_r = 0.0;  self._last_vel_r = 0.0

        # Velocity-mode integrated wheel position (phi dot integral)
        self._phi1_pos = 0.0
        self._phi2_pos = 0.0

        # EMA filter states
        self._roll_filt      = 0.0
        self._pitch_filt     = 0.0
        self._rolldot_filt   = 0.0
        self._pitchdot_filt  = 0.0
        self._phi1dot_filt   = 0.0
        self._phi2dot_filt   = 0.0

        self._dbg_count = 0

    def set_velocity_reference(self, vx_ms: float, vy_ms: float):
        self._vref1 = vx_ms
        self._vref2 = vy_ms

    def _calibrate(self, n=50):
        """Average roll/pitch over n samples (preserved), then record encoder zeros."""
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

        # Record encoder position zeros for phi state (current mode)
        try:
            pos_w, _, pos_r, _ = self.robot.readback()
            self._phi_w_zero = pos_w
            self._phi_r_zero = pos_r
            print(f"  phi zeros: wheel={pos_w:.4f} rad  roller={pos_r:.4f} rad")
        except Exception as exc:
            print(f"  [WARN] encoder zero read failed ({exc}); using 0.0")
            self._phi_w_zero = 0.0
            self._phi_r_zero = 0.0

        # Reset integrated position
        self._phi1_pos = 0.0
        self._phi2_pos = 0.0

    def run(self):
        t = threading.Thread(target=_imu_thread, args=(self.imu,), daemon=True)
        t.start()

        print(f"[ctrl] mode = {CONTROL_MODE!r}")
        if CONTROL_MODE == "current":
            print(f"[ctrl] K_PITCH = {np.round(K_PITCH, 4)}")
            print(f"[ctrl] K_ROLL  = {np.round(K_ROLL, 4)}")
            print(f"[ctrl] TAU_MAX = {TAU_MAX:.2f} Nm  |  soft-start {SOFTSTART_STEPS} steps")

        print("Waiting for IMU data...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond — aborting")
            self.robot.close()
            return

        self._calibrate()

        if CONTROL_MODE == "current":
            self.robot.setTorqueMode()
            self.robot.setCurrentLimit(min(I_LIM_A, I_TEST_A))
            self.robot.setTorque(0.0, 0.0)  # clear any stale BEAR goal_iq before enabling torque
        else:
            self.robot.setVelocityMode()
            self.robot.setCurrentLimit(I_LIM_A)
            self.robot.setVelocity(0.0, 0.0)

        self.robot.toggleTorque(1)
        self._softstart_cnt = 0    # reset ramp on every enable
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
            if CONTROL_MODE == "current":
                self.robot.setTorque(0.0, 0.0)
                time.sleep(0.02)
            else:
                self.robot.setVelocity(0.0, 0.0)
                time.sleep(0.02)
            self.robot.toggleTorque(0)
            self.robot.setTorqueMode()   # safe default state on exit
            self.robot.close()

    # ── Control step ─────────────────────────────────────────────────────────
    def _step(self):
        self._dbg_count += 1
        euler, gyro = self.imu.get()

        # Body angles and rates (calibration offset subtracted)
        roll     = ROLL_EU_SIGN  * math.radians(euler[ROLL_EU_IDX])  - self._roll_offset
        pitch    = PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX]) - self._pitch_offset
        rolldot  = ROLLDOT_SIGN  * gyro[ROLLDOT_IDX]
        pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]
        raw_roll = roll
        raw_pitch = pitch
        raw_rolldot = rolldot
        raw_pitchdot = pitchdot

        # Fall-stop (both modes)
        if abs(roll) > math.radians(FALL_DEG) or abs(pitch) > math.radians(FALL_DEG):
            if CONTROL_MODE == "current":
                self.robot.setTorque(0.0, 0.0)
            else:
                self.robot.setVelocity(0.0, 0.0)
            return

        # Encoder readback (same thread as command — no lock contention)
        try:
            pos_w, vel_w, pos_r, vel_r = self.robot.readback()
            self._last_pos_w = pos_w;  self._last_vel_w = vel_w
            self._last_pos_r = pos_r;  self._last_vel_r = vel_r
        except Exception:
            pos_w = self._last_pos_w;  vel_w = self._last_vel_w
            pos_r = self._last_pos_r;  vel_r = self._last_vel_r

        phi1dot = vel_w   # wheel motor velocity [rad/s] (sign already corrected in readback)
        phi2dot = vel_r   # roller motor velocity [rad/s]

        # EMA low-pass filters (preserved)
        self._roll_filt     = EMA_ALPHA_ANG  * self._roll_filt     + (1 - EMA_ALPHA_ANG)  * roll
        self._pitch_filt    = EMA_ALPHA_ANG  * self._pitch_filt    + (1 - EMA_ALPHA_ANG)  * pitch
        self._rolldot_filt  = EMA_ALPHA_RATE * self._rolldot_filt  + (1 - EMA_ALPHA_RATE) * rolldot
        self._pitchdot_filt = EMA_ALPHA_RATE * self._pitchdot_filt + (1 - EMA_ALPHA_RATE) * pitchdot
        self._phi1dot_filt  = EMA_ALPHA_PHI  * self._phi1dot_filt  + (1 - EMA_ALPHA_PHI)  * phi1dot
        self._phi2dot_filt  = EMA_ALPHA_PHI  * self._phi2dot_filt  + (1 - EMA_ALPHA_PHI)  * phi2dot

        roll     = self._roll_filt;     pitch    = self._pitch_filt
        rolldot  = self._rolldot_filt;  pitchdot = self._pitchdot_filt
        phi1dot  = self._phi1dot_filt;  phi2dot  = self._phi2dot_filt

        # Encoder-based wheel position (relative to calibration zero)
        # PHI_POS_SIGN_W = -1.0: negates pos to match sign of negated vel_w.
        # If robot jolts on enable, flip PHI_POS_SIGN_W to +1.0.
        phi_w = PHI_POS_SIGN_W * (pos_w - self._phi_w_zero)
        phi_r_motor = PHI_POS_SIGN_R * (pos_r - self._phi_r_zero)
        phi_r = N_ROLLER * phi_r_motor   # contact angle for roll LQR

        # Velocity-mode integrated position (preserved for K14/K24 terms)
        self._phi1_pos += phi1dot * CTRL_DT
        self._phi2_pos += phi2dot * CTRL_DT

        # ── CURRENT MODE — LQR full-state feedback ────────────────────────────
        if CONTROL_MODE == "current":
            # Soft-start ramp: 0 → 1 over SOFTSTART_STEPS
            ramp = self._softstart_cnt / SOFTSTART_STEPS
            if self._softstart_cnt < SOFTSTART_STEPS:
                self._softstart_cnt += 1

            # State vectors — x = [φ, θ, φ̇, θ̇].
            # Start with encoder feedback disabled; angle/rate signs must be
            # stable before adding phi/phidot station-keeping terms.
            if USE_ENCODER_FEEDBACK_CURRENT:
                x_pitch = np.array([phi_w, pitch, phi1dot, pitchdot])
                x_roll  = np.array([phi_r, roll, N_ROLLER * phi2dot, rolldot])
            else:
                x_pitch = np.array([0.0, pitch, 0.0, pitchdot])
                x_roll  = np.array([0.0, roll,  0.0, rolldot])

            # LQR law: u = -K @ x  (u in model convention = "positive → forward wheel")
            u_pitch_raw = -float(K_PITCH @ x_pitch)
            u_roll_raw  = -float(K_ROLL  @ x_roll)
            u_pitch = u_pitch_raw
            u_roll  = u_roll_raw

            # Saturate then apply soft-start ramp.
            u_pitch = float(np.clip(u_pitch, -TAU_MAX, TAU_MAX)) * ramp
            u_roll  = float(np.clip(u_roll,  -TAU_MAX, TAU_MAX)) * ramp

            if TORQUE_MIX_MODE == "control_ref":
                tau_wheel_motor = u_pitch
                tau_roller_motor = u_roll - u_pitch
            elif TORQUE_MIX_MODE == "independent":
                tau_wheel_motor = u_pitch
                tau_roller_motor = u_roll
            else:
                raise ValueError(f"unknown TORQUE_MIX_MODE={TORQUE_MIX_MODE!r}")
            tau_wheel_motor = float(np.clip(tau_wheel_motor, -TAU_MAX, TAU_MAX))
            tau_roller_motor = float(np.clip(tau_roller_motor, -TAU_MAX, TAU_MAX))

            self.robot.setTorque(tau_wheel_motor, tau_roller_motor)

            if self._dbg_count % 20 == 0:
                print(
                    f"\n"
                    f"  angle  : roll={math.degrees(roll):+6.2f}°  pitch={math.degrees(pitch):+6.2f}°\n"
                    f"  rate   : rdot={rolldot:+5.2f}  pdot={pitchdot:+5.2f}  rad/s\n"
                    f"  raw    : roll={math.degrees(raw_roll):+6.2f}°"
                    f" pitch={math.degrees(raw_pitch):+6.2f}°"
                    f" rdot={raw_rolldot:+5.2f} pdot={raw_pitchdot:+5.2f}\n"
                    f"  phi    : φ_w={phi_w:+7.3f} rad  φ_r_contact={phi_r:+7.3f} rad"
                    f"  enc_fb={USE_ENCODER_FEEDBACK_CURRENT}\n"
                    f"  lqr    : u_pitch={u_pitch:+6.3f} Nm"
                    f" (raw {u_pitch_raw:+6.3f})"
                    f"  u_roll={u_roll:+6.3f} Nm (raw {u_roll_raw:+6.3f})"
                    f"  mix={TORQUE_MIX_MODE}"
                    f"  motor=(wheel {tau_wheel_motor:+6.3f},"
                    f" roller {tau_roller_motor:+6.3f}) Nm  ramp={ramp:.2f}",
                    flush=True
                )

        # ── VELOCITY MODE — hand-tuned PD (preserved from control2.py) ────────
        else:
            # Angle integral — station keeping (anti-windup)
            int_max = INT_VEL_LIM / max(KI_PITCH, 1e-6)
            self._pitch_int = float(np.clip(
                self._pitch_int + pitch * CTRL_DT, -int_max, int_max))
            int_max_r = INT_VEL_LIM / max(KI_ROLL, 1e-6)
            self._roll_int = float(np.clip(
                self._roll_int + roll * CTRL_DT, -int_max_r, int_max_r))

            # Wheel (motor 2, longitudinal): PD on pitch + φ̇ + φ position
            v_wheel = -(K11 * pitch + K12 * pitchdot + K13 * phi1dot
                        + K14 * self._phi1_pos + KI_PITCH * self._pitch_int)

            # Roller (motor 1, lateral): PD on roll ×N_ROLLER + φ̇ + φ position
            v_roller = ((K21 * roll + K22 * rolldot + KI_ROLL * self._roll_int) * N_ROLLER
                        - K23 * phi2dot - K24 * self._phi2_pos)

            v_wheel  = float(np.clip(v_wheel,  -VEL_MAX, VEL_MAX))
            v_roller = float(np.clip(v_roller, -VEL_MAX, VEL_MAX))
            self.robot.setVelocity(v_wheel, v_roller - v_wheel)

            # Gain-ratio sanity check (print once at startup, then every 1500 steps)
            if self._dbg_count == 1 or self._dbg_count % 1500 == 0:
                print(
                    f"\n  [VEL-MODE gain ratios vs LQR optimal]"
                    f"\n  PITCH  K_φ/K_θ  hand={K14/K11:.5f}  LQR={K_PITCH[0]/K_PITCH[1]:.5f}"
                    f"\n         K_θ̇/K_θ  hand={K12/K11:.5f}  LQR={K_PITCH[3]/K_PITCH[1]:.5f}"
                    f"\n         K_φ̇/K_θ  hand={K13/K11:.5f}  LQR={K_PITCH[2]/K_PITCH[1]:.5f}"
                    f"\n  ROLL   K_φ/K_θ  hand={K24/(K21*N_ROLLER):.5f}  LQR={K_ROLL[0]/K_ROLL[1]:.5f}"
                    f"\n         K_θ̇/K_θ  hand={K22/(K21*N_ROLLER):.5f}  LQR={K_ROLL[3]/K_ROLL[1]:.5f}",
                    flush=True
                )

            if self._dbg_count % 20 == 0:
                print(
                    f"\n"
                    f"  angle  : roll={math.degrees(roll):+6.2f}°  pitch={math.degrees(pitch):+6.2f}°\n"
                    f"  rate   : rdot={rolldot:+5.2f}  pdot={pitchdot:+5.2f}  rad/s\n"
                    f"  phi    : φ_w={phi_w:+7.3f} rad  φ_r_motor={phi_r_motor:+7.3f} rad\n"
                    f"  integ  : pitch_i={KI_PITCH * self._pitch_int:+5.3f}"
                    f"  roll_i={KI_ROLL * self._roll_int:+5.3f}  rad/s\n"
                    f"  vcmd   : wheel={v_wheel:+6.2f}  roller={v_roller:+6.2f}"
                    f"  motor_roller={v_roller - v_wheel:+6.2f}  rad/s",
                    flush=True
                )


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = Controller()
    # ctrl.set_velocity_reference(vx_ms, vy_ms)   # optional motion command
    ctrl.run()
