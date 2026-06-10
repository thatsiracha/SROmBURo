"""
SrOmBURo PID Torque Controller — Raspberry Pi
Uses: MicrostrainIMU (microstrain_imu.py) + Omburo (Omburo.py)

Naming convention matches the OmBURo paper (Shen & Hong, arXiv:2001.07856):
    roll  (θ₁) — longitudinal tilt, forward/backward — motor id2 (wheel)
    pitch (θ₂) — lateral tilt,      side-to-side      — motor id1 (roller)

Note: this is the OPPOSITE of standard robotics convention but matches
the paper. The previous code had these swapped.

Control law per axis:
    outer (50 Hz): cmd_angle = Kp_pos·pos_err − Kd_pos·wheel_vel
    inner (500 Hz): error = cmd_angle − angle
    integral += error · dt                        (anti-windup clamped)
    torque   = Kp·error + Ki·integral + Kd·rate + Kv·wheel_velocity

Motor mixing (from paper §IV, BEAR wiring):
    motor id2 torque = roll_torque
    motor id1 torque = roll_torque − pitch_torque

Tuning order:
    1. KP_ROLL = 10, KD_ROLL = 0.2, KI_ROLL = 0  — get longitudinal balance
    2. KP_PITCH = 10, KD_PITCH = 0.2, KI_PITCH = 0 — get lateral balance
    3. Raise KP until it resists tipping, raise KD to kill oscillation
    4. Add KI (0.1–0.5) only once KP/KD are stable
    5. Add KV (0.2–1.0) to prevent runaway wheel spin
"""

import math
import struct
import sys
import threading
import time
import os

import numpy as np
import serial
from pybear import Manager

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Omburo import Omburo



# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

class Config:
    # Ports
    IMU_PORT    = "/dev/ttyACM0"
    IMU_BAUD    = 115200
    IMU_RATE_HZ = 200          # IMU sample rate — must divide 500 evenly

    # Loop
    CTRL_HZ = 500
    CTRL_DT = 1.0 / CTRL_HZ

    # Physical
    N_ROLLER = 4.0   # roller gear coupling (φ̇₂ = N·(φ̇_wheel + φ̇_roller))

    # ── IMU axis mapping ──────────────────────────────────────────────────────
    # Paper convention (confirmed by hardware test):
    #   roll  (longitudinal, θ₁) = euler[1], sign = -1
    #   pitch (lateral,      θ₂) = euler[0], sign = +1
    ROLL_EU_IDX   = 1;  ROLL_EU_SIGN   =  -1.0   # longitudinal — was PITCH
    PITCH_EU_IDX  = 0;  PITCH_EU_SIGN  =  1.0   # lateral      — was ROLL

    ROLLDOT_IDX   = 1;  ROLLDOT_SIGN   = -1.0   # ωy → θ̇₁ (longitudinal rate)
    PITCHDOT_IDX  = 0;  PITCHDOT_SIGN  =  1.0   # ωx → θ̇₂ (lateral rate)

    # ── PID gains — ROLL axis (longitudinal, forward/backward) ───────────────
    KP_ROLL  =  50.0   # Nm/rad
    KI_ROLL  =  0.1   # Nm/(rad·s) — start at 0, add slowly
    KD_ROLL  =  2.5   # Nm·s/rad   (uses gyro directly, not finite diff)
    KV_ROLL  =  0.0   # Nm/(rad/s) — wheel velocity damping

    # ── PID gains — PITCH axis (lateral, side-to-side) ───────────────────────
    KP_PITCH =  50.0
    KI_PITCH =  0.1
    KD_PITCH =  2.5
    KV_PITCH =  0.0

    # ── Integrator anti-windup ────────────────────────────────────────────────
    INT_CAP_ROLL  = 0.3   # Nm — max integral contribution
    INT_CAP_PITCH = 0.3

    # ── Safety ────────────────────────────────────────────────────────────────
    FALL_DEG     = 40.0   # cut motors if tilt exceeds this [deg]
    MAX_TORQUE   = 2.0    # Nm per motor (BEAR limit: 1.5 A × kt 0.35 = 0.525 Nm)
    MIN_TORQUE   = 0.00   # Nm — below this motors don't move; send 0

    # ── EMA low-pass filter coefficients ─────────────────────────────────────
    # Higher α → more smoothing → more lag. Tune for noise/responsiveness.
    EMA_ANG  = 0.0   # angle  (~8 Hz cutoff at 500 Hz loop)
    EMA_RATE = 0.0   # gyro rate
    EMA_VEL  = 0.15   # wheel velocity (encoder noisier than gyro)

    # ── Outer position loop (50 Hz within 500 Hz inner loop) ─────────────────
    OUTER_LOOP_DIVISOR = 1
    KP_POSITION        = 0.01
    KD_POSITION        = 0.0
    MAX_TARGET_ANGLE   = 0.05   # rad (~5.7°) — clamp outer-loop lean command
    MAX_ANGLE_RATE = 0.25  # Max change of 0.25 rad (~14.3 degrees) per second

    # ── Calibration ───────────────────────────────────────────────────────────
    CALIB_SAMPLES = 100   # IMU samples to average for offset

    # ── Debug ─────────────────────────────────────────────────────────────────
    PRINT_EVERY = 50   # print every N control ticks (~10 Hz at 500 Hz)


# ══════════════════════════════════════════════════════════════════════════════
#  MIP (MicroStrain Inertial Protocol) — IMU driver
# ══════════════════════════════════════════════════════════════════════════════

MIP_SYNC1 = 0x75;  MIP_SYNC2 = 0x65
DESC_BASE = 0x01;  DESC_3DM  = 0x0C;  DESC_IMU = 0x80
FIELD_GYRO = 0x05; FIELD_EULER = 0x0C
MIP_BASE_RATE = 500

def _fletcher(data: bytes) -> tuple[int, int]:
    b1 = b2 = 0
    for b in data:
        b1 = (b1 + b) & 0xFF
        b2 = (b2 + b1) & 0xFF
    return b1, b2

def _build_field(desc: int, data: bytes = b"") -> bytes:
    return bytes([len(data) + 2, desc]) + data

def _build_packet(desc_set: int, fields: list[bytes]) -> bytes:
    payload = b"".join(fields)
    header  = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
    body    = header + payload
    cs1, cs2 = _fletcher(body)
    return body + bytes([cs1, cs2])

def _cmd_ping() -> bytes:
    return _build_packet(DESC_BASE, [_build_field(0x01)])

def _cmd_imu_format(rate_hz: int, field_list: list[int]) -> bytes:
    dec  = max(1, MIP_BASE_RATE // rate_hz)
    data = bytes([0x01, len(field_list)])
    for f in field_list:
        data += bytes([f, dec >> 8, dec & 0xFF])
    return _build_packet(DESC_3DM, [_build_field(0x08, data)])

def _cmd_imu_stream(enable: bool) -> bytes:
    data = bytes([0x01, 0x01, 0x01 if enable else 0x00])
    return _build_packet(DESC_3DM, [_build_field(0x11, data)])

def _read_mip_packet(ser: serial.Serial,
                     timeout: float = 2.0) -> tuple[int, bytes] | None:
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

def _imu_ack(ser: serial.Serial, pkt: bytes,
             cmd_desc: int, retries: int = 3) -> bool:
    for _ in range(retries):
        ser.reset_input_buffer()
        ser.write(pkt)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            r = _read_mip_packet(ser, timeout=deadline - time.time())
            if r is None:
                break
            ds, pl = r
            if ds == DESC_IMU:
                continue
            if len(pl) >= 4 and pl[1] == 0xF1:
                if pl[2] == cmd_desc and pl[3] == 0x00:
                    return True
                break
    return False

def _parse_imu_payload(payload: bytes) -> tuple:
    """Returns (euler_deg_tuple, gyro_rads_tuple) or (None, None)."""
    euler = gyro = None
    i = 0
    while i + 1 < len(payload):
        flen  = payload[i]
        fdesc = payload[i + 1]
        fdata = payload[i + 2: i + flen]
        if fdesc == FIELD_EULER and len(fdata) >= 12:
            r, p, y   = struct.unpack(">fff", fdata[:12])
            euler = (math.degrees(r), math.degrees(p), math.degrees(y))
        elif fdesc == FIELD_GYRO and len(fdata) >= 12:
            gyro = struct.unpack(">fff", fdata[:12])
        i += max(flen, 1)
    return euler, gyro


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD-SAFE IMU STATE
# ══════════════════════════════════════════════════════════════════════════════

class IMUState:
    """Shared memory between IMU reader thread and control loop."""

    def __init__(self):
        self._lock   = threading.Lock()
        self.euler   = (0.0, 0.0, 0.0)   # (roll_deg, pitch_deg, yaw_deg)
        self.gyro    = (0.0, 0.0, 0.0)   # (ωx, ωy, ωz) rad/s
        self.updated = threading.Event()

    def push(self, euler=None, gyro=None):
        with self._lock:
            if euler is not None: self.euler = euler
            if gyro  is not None: self.gyro  = gyro
        self.updated.set()

    def get(self) -> tuple:
        with self._lock:
            return self.euler, self.gyro


def _imu_reader_thread(state: IMUState, cfg: Config):
    """Runs in a daemon thread. Reconnects automatically on serial errors."""
    try:
        with serial.Serial(cfg.IMU_PORT, cfg.IMU_BAUD,
                           timeout=0.05, dsrdtr=False, rtscts=False) as ser:
            time.sleep(1.5)
            ser.reset_input_buffer()

            if not _imu_ack(ser, _cmd_ping(), 0x01):
                print("[IMU] No ping response — check cable and power"); return

            _imu_ack(ser, _cmd_imu_stream(False), 0x11)   # stop any old stream
            time.sleep(0.1); ser.reset_input_buffer()

            if not _imu_ack(ser,
                            _cmd_imu_format(cfg.IMU_RATE_HZ,
                                            [FIELD_EULER, FIELD_GYRO]), 0x08):
                print("[IMU] Format config failed"); return

            if not _imu_ack(ser, _cmd_imu_stream(True), 0x11):
                print("[IMU] Stream enable failed"); return

            print("[IMU] Stream active")
            while True:
                r = _read_mip_packet(ser, timeout=0.5)
                if r is None:
                    continue
                ds, pl = r
                if ds != DESC_IMU:
                    continue
                euler, gyro = _parse_imu_payload(pl)
                state.push(euler=euler, gyro=gyro)

    except Exception as exc:
        print(f"[IMU] Thread error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
#  PID CONTROLLER (one instance per axis)
# ══════════════════════════════════════════════════════════════════════════════

class PIDTorque:
    """
    Single-axis PID controller. Output is torque in Nm.

    Uses gyroscope angular rate directly for the derivative term — avoids
    noisy finite-differencing of the filtered angle estimate.
    """

    DERIV_LP_ALPHA = 0.7   # low-pass on derivative output (0 = none, 1 = heavy)

    def __init__(self, kp: float, ki: float, kd: float, kv: float,
                 int_cap: float, dt: float):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.kv      = kv       # wheel velocity damping gain
        self.int_cap = int_cap
        self.dt      = dt

        self._integral  = 0.0
        self._deriv_lp  = 0.0

    def compute(self, angle: float, rate: float, wheel_vel: float,
                target_angle: float = 0.0) -> float:
        """
        angle        : tilt angle (rad) — positive means leaning in one direction
        rate         : angular rate from gyro (rad/s)
        wheel_vel    : wheel/roller encoder velocity (rad/s)
        target_angle : desired lean from outer position loop (rad)
        Returns      : torque command (Nm) — sign convention: positive torque
                       opposes positive tilt.
        """
        error = target_angle - angle

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral = float(np.clip(
            self._integral + error * self.dt,
            -self.int_cap / max(self.ki, 1e-9),
            +self.int_cap / max(self.ki, 1e-9),
        ))
        i = self.ki * self._integral

        # Derivative from gyro (low-pass filtered)
        raw_d = -self.kd * rate   # negative: rate in same direction as tilt
        self._deriv_lp = (self.DERIV_LP_ALPHA * self._deriv_lp +
                          (1.0 - self.DERIV_LP_ALPHA) * raw_d)
        d = self._deriv_lp

        # Velocity damping
        v = -self.kv * wheel_vel

        return p + i + d + v

    def reset(self):
        self._integral = 0.0
        self._deriv_lp = 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN CONTROLLER
# ══════════════════════════════════════════════════════════════════════════════

class OmBUROPIDController:
    """
    PID torque balancing controller for SrOmBURo.

    Naming follows the paper:
        roll  = longitudinal axis (forward/backward tilt, θ₁)
        pitch = lateral axis      (side-to-side tilt,    θ₂)
    """

    def __init__(self):
        from pybear import Manager

        self.cfg   = Config()
        self.imu   = IMUState()
        self.robot = Omburo()

        self.pid_roll  = PIDTorque(
            kp=self.cfg.KP_ROLL,  ki=self.cfg.KI_ROLL,
            kd=self.cfg.KD_ROLL,  kv=self.cfg.KV_ROLL,
            int_cap=self.cfg.INT_CAP_ROLL, dt=self.cfg.CTRL_DT
        )
        self.pid_pitch = PIDTorque(
            kp=self.cfg.KP_PITCH, ki=self.cfg.KI_PITCH,
            kd=self.cfg.KD_PITCH, kv=self.cfg.KV_PITCH,
            int_cap=self.cfg.INT_CAP_PITCH, dt=self.cfg.CTRL_DT
        )

        # EMA filter states — roll axis (longitudinal)
        self._roll_filt      = 0.0
        self._rolldot_filt   = 0.0
        self._vel_roll_filt  = 0.0   # motor id2 velocity

        # EMA filter states — pitch axis (lateral)
        self._pitch_filt     = 0.0
        self._pitchdot_filt  = 0.0
        self._vel_pitch_filt = 0.0   # motor id1+id2 velocity

        # Calibration offsets
        self._roll_offset  = 0.0
        self._pitch_offset = 0.0

        # Fallback encoder values
        self._last_vel_w = 0.0
        self._last_vel_r = 0.0

        # Outer position loop state
        self.pos_roll         = 0.0
        self.pos_pitch        = 0.0
        self.target_pos_roll  = 0.0
        self.target_pos_pitch = 0.0
        self.cmd_angle_roll   = 0.0
        self.cmd_angle_pitch  = 0.0

        self._running = False

    # ── Public API ─────────────────────────────────────────────────────────────

    def run(self):
        """Start IMU thread, calibrate, then enter 500 Hz control loop."""
        print("=" * 60)
        print("  SrOmBURo PID Torque Controller")
        print("  Axes: roll=longitudinal(θ₁)  pitch=lateral(θ₂)")
        print("=" * 60)

        # Start IMU background thread
        t = threading.Thread(target=_imu_reader_thread,
                             args=(self.imu, self.cfg), daemon=True)
        t.start()

        print("[Init] Waiting for first IMU packet...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond in 8 s — check cable"); return

        # Switch to torque/current control mode
        self._set_torque_mode()

        # Calibrate static offsets
        self._calibrate()

        #Turns on torque for motors
        self.robot.toggleTorque(1)
        time.sleep(0.2)

        #Sets motor to torque mode (0)
        self.robot.setTorqueMode()

        self.pos_roll  = 0.0
        self.pos_pitch = 0.0

        print(f"\n[Init] Loop rate : {self.cfg.CTRL_HZ} Hz")
        print(f"[Init] KP_ROLL={self.cfg.KP_ROLL}  KD_ROLL={self.cfg.KD_ROLL}  "
              f"KI_ROLL={self.cfg.KI_ROLL}")
        print(f"[Init] KP_PITCH={self.cfg.KP_PITCH}  KD_PITCH={self.cfg.KD_PITCH}  "
              f"KI_PITCH={self.cfg.KI_PITCH}")
        print(f"[Init] MAX_TORQUE={self.cfg.MAX_TORQUE} Nm  "
              f"FALL_STOP={self.cfg.FALL_DEG}°")
        print("[Init] Balancing active — Ctrl+C to stop\n")

        self._running = True
        self._control_loop()

    # ── Hardware helpers ───────────────────────────────────────────────────────

    def _set_torque_mode(self):
        """Switch BEAR motors to current (torque) control mode = 0."""
        try:
            self.robot.bear.set_mode((self.robot.bear.id_wheel,  0),
                                     (self.robot.bear.id_roller, 0))
        except AttributeError:
            # Fallback: access bear internals directly
            try:
                from pybear import Manager
                self.robot.bear.bear.set_mode(
                    (2, 0), (1, 0)   # id_wheel=2, id_roller=1
                )
            except Exception:
                pass   # if mode switching fails, setTorque() still works

    # ── Calibration ────────────────────────────────────────────────────────────

    def _calibrate(self):
        """
        Average IMU output over N samples to compute static angle offsets.
        Robot must be upright and still during calibration.
        """
        n = self.cfg.CALIB_SAMPLES
        print(f"[Calib] Averaging {n} IMU samples — hold robot STILL and UPRIGHT...")
        roll_sum = pitch_sum = 0.0
        collected = 0
        while collected < n:
            self.imu.updated.wait(); self.imu.updated.clear()
            euler, _ = self.imu.get()
            roll_sum  += (self.cfg.ROLL_EU_SIGN  *
                          math.radians(euler[self.cfg.ROLL_EU_IDX]))
            pitch_sum += (self.cfg.PITCH_EU_SIGN *
                          math.radians(euler[self.cfg.PITCH_EU_IDX]))
            collected += 1

        self._roll_offset  = roll_sum  / n
        self._pitch_offset = pitch_sum / n
        self.pid_roll.reset()
        self.pid_pitch.reset()

        print(f"[Calib] roll_offset  = {math.degrees(self._roll_offset):+.3f}°  "
              f"pitch_offset = {math.degrees(self._pitch_offset):+.3f}°")

    # ── 500 Hz control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        cfg     = self.cfg
        t_next  = time.perf_counter()
        tick    = 0
        hz_tick = 0
        hz_t0   = time.perf_counter()

        try:
            while self._running:
                # Precise timing: sleep most of the interval, busy-wait the rest
                now  = time.perf_counter()
                wait = t_next - now
                if wait > 0:
                    time.sleep(wait * 0.85)
                    while time.perf_counter() < t_next:
                        pass
                t_next += cfg.CTRL_DT
                tick   += 1
                hz_tick += 1

                self._step(tick)

                if hz_tick >= 250:
                    elapsed = time.perf_counter() - hz_t0
                    print(f"\r[loop] {hz_tick / elapsed:.1f} Hz", end="", flush=True)
                    hz_tick = 0
                    hz_t0   = time.perf_counter()

        except KeyboardInterrupt:
            print("\n[Controller] Stopping...")
        finally:
            self._stop()

    # ── Single control step ─────────────────────────────────────────────────────

    def _step(self, tick: int):
        cfg = self.cfg
        euler, gyro = self.imu.get()

        # ── 1. Sensor → body angles (paper convention) ──────────────────────
        roll  = (cfg.ROLL_EU_SIGN  * math.radians(euler[cfg.ROLL_EU_IDX])
                 - self._roll_offset)
        pitch = (cfg.PITCH_EU_SIGN * math.radians(euler[cfg.PITCH_EU_IDX])
                 - self._pitch_offset)

        rolldot  = cfg.ROLLDOT_SIGN  * gyro[cfg.ROLLDOT_IDX]
        pitchdot = cfg.PITCHDOT_SIGN * gyro[cfg.PITCHDOT_IDX]

        # ── 2. Fall-stop ──────────────────────────────────────────────────────
        fall = math.radians(cfg.FALL_DEG)
        if abs(roll) > fall or abs(pitch) > fall:
            self.robot.setTorque(0.0, 0.0)
            if tick % cfg.PRINT_EVERY == 0:
                print(f"\n[SAFETY] Fall detected — "
                      f"roll={math.degrees(roll):.1f}°  "
                      f"pitch={math.degrees(pitch):.1f}°  — MOTORS OFF")
            return

        # ── 3. Encoder readback ───────────────────────────────────────────────
        try:
            _, vel_w, _, vel_r = self.robot.readback()
            self._last_vel_w = vel_w
            self._last_vel_r = vel_r
        except Exception:
            vel_w = self._last_vel_w
            vel_r = self._last_vel_r

        # Motor wiring → axis velocities (paper §IV):
        #   roll  axis velocity = motor id2 (wheel)
        #   pitch axis velocity = motor id2 + motor id1 (roller coupling)
        vel_roll_raw  = vel_w
        vel_pitch_raw = vel_w + vel_r

        self.pos_roll  += vel_roll_raw  * cfg.CTRL_DT
        self.pos_pitch += vel_pitch_raw * cfg.CTRL_DT

        # ── 4. EMA low-pass filters ───────────────────────────────────────────
        a_ang  = cfg.EMA_ANG
        a_rate = cfg.EMA_RATE
        a_vel  = cfg.EMA_VEL

        self._roll_filt      = a_ang  * self._roll_filt      + (1 - a_ang)  * roll
        self._pitch_filt     = a_ang  * self._pitch_filt     + (1 - a_ang)  * pitch
        self._rolldot_filt   = a_rate * self._rolldot_filt   + (1 - a_rate) * rolldot
        self._pitchdot_filt  = a_rate * self._pitchdot_filt  + (1 - a_rate) * pitchdot
        self._vel_roll_filt  = a_vel  * self._vel_roll_filt  + (1 - a_vel)  * vel_roll_raw
        self._vel_pitch_filt = a_vel  * self._vel_pitch_filt + (1 - a_vel)  * vel_pitch_raw

        roll_f      = self._roll_filt
        pitch_f     = self._pitch_filt
        rolldot_f   = self._rolldot_filt
        pitchdot_f  = self._pitchdot_filt
        vel_roll_f  = self._vel_roll_filt
        vel_pitch_f = self._vel_pitch_filt

        # ── 4b. Outer position loop (50 Hz) ───────────────────────────────────
        
        # 1. Calculate the raw desired target angle from the outer PD loop
        desired_roll  = (cfg.KP_POSITION * err_pos_roll)  - (cfg.KD_POSITION * vel_roll_f)
        desired_pitch = (cfg.KP_POSITION * err_pos_pitch) - (cfg.KD_POSITION * vel_pitch_f)

        # 2. Clamp the desired angles to our absolute safety ceiling
        desired_roll  = max(-cfg.MAX_TARGET_ANGLE, min(desired_roll,  cfg.MAX_TARGET_ANGLE))
        desired_pitch = max(-cfg.MAX_TARGET_ANGLE, min(desired_pitch, cfg.MAX_TARGET_ANGLE))

        # 3. Calculate the maximum amount the angle is allowed to change in a single 2ms tick
        max_change_per_tick = cfg.MAX_ANGLE_RATE * cfg.CTRL_DT

        # 4. Apply the Slew Rate Limit (only move toward the desired angle by the max allowed step)
        diff_roll  = desired_roll  - self.cmd_angle_roll
        diff_pitch = desired_pitch - self.cmd_angle_pitch

        self.cmd_angle_roll  += max(-max_change_per_tick, min(diff_roll,  max_change_per_tick))
        self.cmd_angle_pitch += max(-max_change_per_tick, min(diff_pitch, max_change_per_tick))

        # ── 5. PID torque computation ─────────────────────────────────────────
        tau_roll  = self.pid_roll.compute(roll_f,  rolldot_f,  vel_roll_f,
                                          target_angle=self.cmd_angle_roll)
        tau_pitch = -self.pid_pitch.compute(pitch_f, pitchdot_f, vel_pitch_f,
                                            target_angle=self.cmd_angle_pitch)

        # ── 6. Motor mixing (paper convention) ───────────────────────────────
        #   motor id2 (wheel)  = roll torque
        #   motor id1 (roller) = roll torque − pitch torque
        tau_motor2 = tau_roll + (tau_pitch / 4) # where 8 is the gear ratio N_ROLLER 
        tau_motor1 = tau_pitch

        # Saturate
        tau_motor2 = float(np.clip(tau_motor2, -cfg.MAX_TORQUE, cfg.MAX_TORQUE))
        tau_motor1 = float(np.clip(tau_motor1, -cfg.MAX_TORQUE, cfg.MAX_TORQUE))

        # Below noise floor → send zero (don't waste current on micro-commands)
        if abs(tau_motor2) < cfg.MIN_TORQUE: tau_motor2 = 0.0
        if abs(tau_motor1) < cfg.MIN_TORQUE: tau_motor1 = 0.0

        # ── 7. Send to motors ─────────────────────────────────────────────────
        # Omburo.setTorque(id_wheel=2, id_roller=1)
        self.robot.setTorque(tau_motor2, tau_motor1)
        

        # ── 8. Debug print (~10 Hz) ───────────────────────────────────────────
        if tick % cfg.PRINT_EVERY == 0:
            print(
                f"\n"
                f"  ROLL  (long): angle={math.degrees(roll_f):+6.2f}°  "
                f"cmd={math.degrees(self.cmd_angle_roll):+6.2f}°  "
                f"rate={math.degrees(rolldot_f):+6.2f}°/s  "
                f"vel={vel_roll_f:+.3f} rad/s  "
                f"τ_roll={tau_roll:+.4f} Nm\n"
                f"  PITCH (lat) : angle={math.degrees(pitch_f):+6.2f}°  "
                f"cmd={math.degrees(self.cmd_angle_pitch):+6.2f}°  "
                f"rate={math.degrees(pitchdot_f):+6.2f}°/s  "
                f"vel={vel_pitch_f:+.3f} rad/s  "
                f"τ_pitch={tau_pitch:+.4f} Nm\n"
                f"  MOTORS      : id2(wheel)={tau_motor2:+.4f} Nm  "
                f"id1(roller)={tau_motor1:+.4f} Nm",
                flush=True
            )

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def _stop(self):
        print("[Controller] Zeroing torques and disabling...")
        try:
            self.robot.setTorque(0.0, 0.0)
            time.sleep(0.1)
            self.robot.toggleTorque(0)
        except Exception:
            pass
        try:
            self.robot.close()
        except Exception:
            pass
        print("[Controller] Done.")


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    ctrl = OmBUROPIDController()
    ctrl.run()