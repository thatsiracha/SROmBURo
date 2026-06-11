"""
SrOmBURo PID Torque Controller — Raspberry Pi
Uses: MicrostrainIMU (microstrain_imu.py) + Omburo (Omburo.py)

Axis and motor direction convention follows control2_Suraj.py:
    roll  — lateral tilt,      euler[0], sign +1
    pitch — longitudinal tilt, euler[1], sign -1

Control law per axis:
    error    = 0 − angle                          (want upright = 0 rad)
    integral += error · dt                        (anti-windup clamped)
    torque   = Kp·error + Ki·integral + Kd·rate + Kv·wheel_velocity

Motor output, matching control2_Suraj.py:
    motor id2 torque = pitch_torque
    motor id1 torque = roll_torque

Tuning order:
    1. KP_ROLL = 10, KD_ROLL = 0.2, KI_ROLL = 0  — get lateral balance
    2. KP_PITCH = 10, KD_PITCH = 0.2, KI_PITCH = 0 — get longitudinal balance
    3. Raise KP until it resists tipping, raise KD to kill oscillation
    4. Add KI (0.1–0.5) only once KP/KD are stable
    5. Add KV (0.2–1.0) to prevent runaway wheel spin
"""

import csv
import html
import math
import os
import struct
import sys
import termios
import threading
import time
import tty

import serial
from pybear import Manager

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from Omburo import Omburo



# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

class Config:
    # Ports
    IMU_PORT    = "/dev/ttyACM0"
    IMU_BAUD    = 115200
    IMU_RATE_HZ = 500         # IMU sample rate — synced 1:1 with control loop

    # Loop
    CTRL_HZ = 500
    CTRL_DT = 1.0 / CTRL_HZ
    COMMAND_RAMP_SEC = 3.0   # linearly ramp motor commands after SPACE start

    # Physical
    N_ROLLER = 4.0   # roller gear coupling (φ̇₂ = N·(φ̇_wheel + φ̇_roller))

    # ── IMU axis mapping ──────────────────────────────────────────────────────
    # Matches control2_Suraj.py: roll=euler[0](+), pitch=euler[1](-).
    ROLL_EU_IDX   = 0;  ROLL_EU_SIGN   =  1.0   # roll  → lateral axis
    PITCH_EU_IDX  = 1;  PITCH_EU_SIGN  = -1.0   # pitch → longitudinal axis

    ROLLDOT_IDX   = 0;  ROLLDOT_SIGN   =  1.0   # ωx → roll rate
    PITCHDOT_IDX  = 1;  PITCHDOT_SIGN  = -1.0   # ωy → pitch rate

    # ── PID gains — ROLL axis (lateral, side-to-side) ─────────────────────────
    KP_ROLL  = 15.0   # Nm/rad
    KI_ROLL  = 0.5   # Nm/(rad·s) — start at 0, add slowly
    KD_ROLL  = 2.0   # Nm·s/rad   (uses gyro directly, not finite diff)
    KV_ROLL  =  0.0   # Nm/(rad/s) — wheel velocity damping

    # ── PID gains — PITCH axis (longitudinal, forward/backward) ───────────────
    KP_PITCH = 15.0
    KI_PITCH = 0.5
    KD_PITCH = 2.0
    KV_PITCH =  0.0

    # ── Integrator anti-windup ────────────────────────────────────────────────
    INT_CAP_ROLL  = 0.3   # Nm — max integral contribution
    INT_CAP_PITCH = 0.3

    # ── Safety ────────────────────────────────────────────────────────────────
    FALL_DEG     = 40.0   # cut motors if tilt exceeds this [deg]
    MAX_TORQUE   = 6    # Nm per motor (BEAR limit: 6.3 A × kt 0.35 = 0.525 Nm)
    MIN_TORQUE   = 0.00   # Nm — below this motors don't move; send 0

    # ── EMA low-pass filter coefficients ─────────────────────────────────────
    # Set to 0.0 to bypass EMA: filtered = raw.
    EMA_ANG  = 0.0    # angle
    EMA_RATE = 0.0    # gyro rate
    EMA_VEL  = 0.0    # wheel velocity

    # ── Calibration ───────────────────────────────────────────────────────────
    CALIB_SAMPLES = 100   # IMU samples to average for offset

    # ── Feedforward Parameters (Gravity & Friction) ───────────────────────────
    # Gravity FF uses the above-axis body mass m_b, not total robot mass.
    M_TOTAL = 3.04       # [kg] total robot mass, reference only
    G_ACCEL = 9.81       # [m/s²]
    M_BODY = 2.42        # [kg] above-axis body mass m_b
    M_WHEEL = 0.620      # [kg] wheel/axis mass m_w
    L_COM_ROLL = 0.7348  # [m] roll-axis to body COM
    L_COM_PITCH = 0.647  # [m] pitch-axis to body COM
    R_ROLL = 0.0142      # [m] roll drive radius
    R_PITCH = 0.102      # [m] pitch drive radius
    R_DRIVE_ROLL = R_ROLL / N_ROLLER  # [m] effective roll radius through roller gear
    R_DRIVE_PITCH = R_PITCH           # [m] pitch direct-drive effective radius

    I_W_ROLL = 0.0       # [kg·m²] roller inertia ignored; TODO: replace if measured
    I_W_PITCH = 0.0030   # [kg·m²] pitch wheel inertia

    M_EFF_ROLL = M_BODY + M_WHEEL + I_W_ROLL / (R_DRIVE_ROLL**2)
    M_EFF_PITCH = 3.33   # [kg] = M_BODY + M_WHEEL + I_W_PITCH / R_PITCH²
    MGL_BODY_ROLL = M_BODY * G_ACCEL * L_COM_ROLL
    MGL_BODY_PITCH = M_BODY * G_ACCEL * L_COM_PITCH

    # Coulomb Friction
    FRIC_ROLL  = 0.5  # Nm
    FRIC_PITCH = 0.3  # Nm

    # tanh 마찰 보상 스케일 (클수록 sign에 가까움, 작을수록 부드러움)
    FRIC_TANH_SCALE = 20.0   # 1/(rad/s)

    # ── Debug ─────────────────────────────────────────────────────────────────
    PRINT_EVERY = 50   # print every N control ticks (~1 Hz at 500 Hz)
    DEBUG_CSV = True


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
            euler = self.euler
            gyro = self.gyro
        return euler, gyro


def _imu_reader_thread(state: IMUState, cfg: Config,
                       stop_event: threading.Event):
    """Runs in a daemon thread. Reconnects automatically on serial errors."""
    try:
        with serial.Serial(cfg.IMU_PORT, cfg.IMU_BAUD,
                           timeout=0.05, dsrdtr=False, rtscts=False) as ser:
            try:
                time.sleep(1.5)
                ser.reset_input_buffer()

                # A previous run may have left the IMU streaming, which can
                # flood the port and hide the ping ACK on the next startup.
                _imu_ack(ser, _cmd_imu_stream(False), 0x11, retries=2)
                time.sleep(0.1); ser.reset_input_buffer()

                if not _imu_ack(ser, _cmd_ping(), 0x01):
                    print("[IMU] No ping response — check cable and power"); return

                if not _imu_ack(ser,
                                _cmd_imu_format(cfg.IMU_RATE_HZ,
                                                [FIELD_EULER, FIELD_GYRO]), 0x08):
                    print("[IMU] Format config failed"); return

                if not _imu_ack(ser, _cmd_imu_stream(True), 0x11):
                    print("[IMU] Stream enable failed"); return

                print("[IMU] Stream active")
                while not stop_event.is_set():
                    r = _read_mip_packet(ser, timeout=0.1)
                    if r is None:
                        continue
                    ds, pl = r
                    if ds != DESC_IMU:
                        continue
                    euler, gyro = _parse_imu_payload(pl)
                    state.push(euler=euler, gyro=gyro)

            finally:
                try:
                    _imu_ack(ser, _cmd_imu_stream(False), 0x11, retries=2)
                    print("[IMU] Stream stopped")
                except Exception:
                    pass

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

    def compute(self, angle: float, rate: float, wheel_vel: float) -> float:
        """
        angle     : tilt angle (rad) — positive means leaning in one direction
        rate      : angular rate from gyro (rad/s)
        wheel_vel : wheel/roller encoder velocity (rad/s)
        Returns   : torque command (Nm) — sign convention: positive torque
                    opposes positive tilt.
        """
        error = -angle   # negative: positive tilt → we want negative torque

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        int_limit = self.int_cap / max(self.ki, 1e-9)
        self._integral = max(-int_limit, min(self._integral + error * self.dt,
                                             int_limit))
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

    Axis and motor direction convention follows control2_Suraj.py:
        roll  = lateral axis
        pitch = longitudinal axis
    """

    def __init__(self):
        from pybear import Manager

        self.cfg   = Config()
        self.imu   = IMUState()
        self.robot = Omburo()
        self._imu_stop = threading.Event()
        self._imu_thread = None

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

        # EMA filter states — roll axis (lateral)
        self._roll_filt      = 0.0
        self._rolldot_filt   = 0.0
        self._vel_roll_filt  = 0.0   # motor id2 velocity

        # EMA filter states — pitch axis (longitudinal)
        self._pitch_filt     = 0.0
        self._pitchdot_filt  = 0.0
        self._vel_pitch_filt = 0.0   # motor id1+id2 velocity

        # Calibration offsets
        self._roll_offset  = 0.0
        self._pitch_offset = 0.0

        # Fallback encoder values
        self._last_vel_w = 0.0
        self._last_vel_r = 0.0

        # Soft-start state
        self._command_ramp_t0 = None

        # CSV debug log
        self._log_file = None
        self._log_writer = None
        self._log_path = None
        self._run_dir = None
        self._log_t0 = None
        if self.cfg.DEBUG_CSV:
            self._open_debug_csv()

        self._running = False

    # ── Public API ─────────────────────────────────────────────────────────────

    def _open_debug_csv(self):
        """Create a timestamped run folder and CSV log."""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._run_dir = os.path.join(base_dir, timestamp)
        os.makedirs(self._run_dir, exist_ok=True)
        self._log_path = os.path.join(self._run_dir, f"control_debug_{timestamp}.csv")
        self._log_t0 = time.perf_counter()
        self._log_file = open(self._log_path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            "tick",
            "time_s",
            "euler_roll_deg",
            "euler_pitch_deg",
            "gyro_roll_rad_s",
            "gyro_pitch_rad_s",
            "roll_offset_deg",
            "pitch_offset_deg",
            "roll_rad",
            "pitch_rad",
            "rolldot_rad_s",
            "pitchdot_rad_s",
            "vel_roll_raw_rad_s",
            "vel_pitch_raw_rad_s",
            "roll_f_rad",
            "pitch_f_rad",
            "roll_f_deg",
            "pitch_f_deg",
            "rolldot_f_rad_s",
            "pitchdot_f_rad_s",
            "vel_roll_f_rad_s",
            "vel_pitch_f_rad_s",
            "tau_fb_roll_nm",
            "tau_fb_pitch_nm",
            "denom_roll",
            "denom_pitch",
            "tau_ff_roll_g_nm",
            "tau_ff_pitch_g_nm",
            "tau_ff_roll_f_nm",
            "tau_ff_pitch_f_nm",
            "tau_ff_roll_nm",
            "tau_ff_pitch_nm",
            "tau_roll_total_nm",
            "tau_pitch_total_nm",
            "tau_motor1_raw_nm",
            "tau_motor1_sat_nm",
            "tau_motor1_ramped_nm",
            "tau_motor1_final_nm",
            "tau_motor2_raw_nm",
            "tau_motor2_sat_nm",
            "tau_motor2_ramped_nm",
            "tau_motor2_final_nm",
            "ramp_scale",
        ])
        self._log_file.flush()

    def _plot_debug_csv(self):
        """Generate SVG plots for the run CSV using only the standard library."""
        if self._log_path is None or self._run_dir is None:
            return
        try:
            with open(self._log_path, newline="") as fh:
                rows = list(csv.DictReader(fh))
        except Exception as exc:
            print(f"[Controller] Plot generation failed: {exc}")
            return

        if not rows:
            print("[Controller] Plot generation skipped: CSV has no data rows")
            return

        plot_specs = [
            ("01_time_roll_angle.svg", "time - roll angle", "roll_f_deg", "Roll angle [deg]"),
            ("02_time_pitch_angle.svg", "time - pitch angle", "pitch_f_deg", "Pitch angle [deg]"),
            ("03_time_feed_forward_roll.svg", "time - feed forward command (roll)", "tau_ff_roll_nm", "Feed forward roll [Nm]"),
            ("04_time_feed_forward_pitch.svg", "time - feed forward command (pitch)", "tau_ff_pitch_nm", "Feed forward pitch [Nm]"),
            ("05_time_pid_roll.svg", "time - PID (roll)", "tau_fb_roll_nm", "PID roll [Nm]"),
            ("06_time_pid_pitch.svg", "time - PID (pitch)", "tau_fb_pitch_nm", "PID pitch [Nm]"),
            ("07_time_friction_roll.svg", "time - friction (roll)", "tau_ff_roll_f_nm", "Friction roll [Nm]"),
            ("08_time_friction_pitch.svg", "time - friction (pitch)", "tau_ff_pitch_f_nm", "Friction pitch [Nm]"),
            ("09_time_total_roll.svg", "time - total (roll)", "tau_roll_total_nm", "Total roll [Nm]"),
            ("10_time_total_pitch.svg", "time - total (pitch)", "tau_pitch_total_nm", "Total pitch [Nm]"),
        ]

        saved = 0
        for filename, title, y_key, y_label in plot_specs:
            points = []
            for row in rows:
                try:
                    x = float(row["time_s"])
                    y = float(row[y_key])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(x) and math.isfinite(y):
                    points.append((x, y))
            if not points:
                continue
            self._write_svg_plot(
                path=os.path.join(self._run_dir, filename),
                title=title,
                points=points,
                x_label="Time [s]",
                y_label=y_label,
            )
            saved += 1

        self._write_svg_dashboard(
            path=os.path.join(self._run_dir, "11_all_plots.svg"),
            title="SrOmBURo control debug summary",
            rows=rows,
            plot_specs=plot_specs,
        )
        saved += 1

        print(f"[Controller] Plot files saved: {self._run_dir} ({saved} SVG)")

    @staticmethod
    def _points_from_rows(rows, y_key):
        points = []
        for row in rows:
            try:
                x = float(row["time_s"])
                y = float(row[y_key])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        return points

    @staticmethod
    def _write_svg_plot(path, title, points, x_label, y_label):
        width, height = 1000, 520
        left, right, top, bottom = 90, 30, 55, 70
        plot_w = width - left - right
        plot_h = height - top - bottom

        if len(points) > 2000:
            step = math.ceil(len(points) / 2000)
            points = points[::step]

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_min == x_max:
            x_min -= 0.5
            x_max += 0.5
        if y_min == y_max:
            pad = max(abs(y_min) * 0.05, 1.0)
            y_min -= pad
            y_max += pad
        else:
            pad = (y_max - y_min) * 0.08
            y_min -= pad
            y_max += pad

        def sx(x):
            return left + (x - x_min) / (x_max - x_min) * plot_w

        def sy(y):
            return top + (y_max - y) / (y_max - y_min) * plot_h

        polyline = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
        h_title = html.escape(title)
        h_x_label = html.escape(x_label)
        h_y_label = html.escape(y_label)

        grid_lines = []
        tick_labels = []
        for i in range(6):
            gx = left + plot_w * i / 5
            x_val = x_min + (x_max - x_min) * i / 5
            grid_lines.append(
                f'<line x1="{gx:.2f}" y1="{top}" x2="{gx:.2f}" y2="{top + plot_h}" stroke="#eeeeee"/>'
            )
            tick_labels.append(
                f'<text x="{gx:.2f}" y="{top + plot_h + 24}" font-size="12" text-anchor="middle">{x_val:.2f}</text>'
            )
        for i in range(6):
            gy = top + plot_h * i / 5
            y_val = y_max - (y_max - y_min) * i / 5
            grid_lines.append(
                f'<line x1="{left}" y1="{gy:.2f}" x2="{left + plot_w}" y2="{gy:.2f}" stroke="#eeeeee"/>'
            )
            tick_labels.append(
                f'<text x="{left - 12}" y="{gy + 4:.2f}" font-size="12" text-anchor="end">{y_val:.3g}</text>'
            )

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{width / 2}" y="28" font-size="22" font-family="Arial, sans-serif" text-anchor="middle">{h_title}</text>
<g font-family="Arial, sans-serif" fill="#222">
{''.join(grid_lines)}
<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" stroke="#333" stroke-width="1.5"/>
<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" stroke="#333" stroke-width="1.5"/>
<polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{polyline}"/>
{''.join(tick_labels)}
<text x="{left + plot_w / 2}" y="{height - 22}" font-size="14" text-anchor="middle">{h_x_label}</text>
<text transform="translate(24 {top + plot_h / 2}) rotate(-90)" font-size="14" text-anchor="middle">{h_y_label}</text>
</g>
</svg>
'''
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(svg)

    @staticmethod
    def _write_svg_dashboard(path, title, rows, plot_specs):
        width, height = 1600, 1800
        margin_x, margin_top, margin_bottom = 80, 70, 55
        gap_x, gap_y = 70, 58
        cols, rows_n = 2, 5
        panel_w = (width - 2 * margin_x - gap_x) / cols
        panel_h = (height - margin_top - margin_bottom - gap_y * (rows_n - 1)) / rows_n

        panels = []
        for idx, (_, panel_title, y_key, y_label) in enumerate(plot_specs):
            points = OmBUROPIDController._points_from_rows(rows, y_key)
            if not points:
                continue
            if len(points) > 1000:
                step = math.ceil(len(points) / 1000)
                points = points[::step]

            col = idx % cols
            row = idx // cols
            x0 = margin_x + col * (panel_w + gap_x)
            y0 = margin_top + row * (panel_h + gap_y)
            left, right, top, bottom = 58, 12, 26, 38
            plot_x0 = x0 + left
            plot_y0 = y0 + top
            plot_w = panel_w - left - right
            plot_h = panel_h - top - bottom

            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            if x_min == x_max:
                x_min -= 0.5
                x_max += 0.5
            if y_min == y_max:
                pad = max(abs(y_min) * 0.05, 1.0)
                y_min -= pad
                y_max += pad
            else:
                pad = (y_max - y_min) * 0.08
                y_min -= pad
                y_max += pad

            def sx(x):
                return plot_x0 + (x - x_min) / (x_max - x_min) * plot_w

            def sy(y):
                return plot_y0 + (y_max - y) / (y_max - y_min) * plot_h

            grid = []
            labels = []
            for i in range(4):
                gx = plot_x0 + plot_w * i / 3
                x_val = x_min + (x_max - x_min) * i / 3
                grid.append(
                    f'<line x1="{gx:.2f}" y1="{plot_y0:.2f}" x2="{gx:.2f}" y2="{plot_y0 + plot_h:.2f}" stroke="#eeeeee"/>'
                )
                labels.append(
                    f'<text x="{gx:.2f}" y="{plot_y0 + plot_h + 18:.2f}" font-size="10" text-anchor="middle">{x_val:.1f}</text>'
                )
            for i in range(4):
                gy = plot_y0 + plot_h * i / 3
                y_val = y_max - (y_max - y_min) * i / 3
                grid.append(
                    f'<line x1="{plot_x0:.2f}" y1="{gy:.2f}" x2="{plot_x0 + plot_w:.2f}" y2="{gy:.2f}" stroke="#eeeeee"/>'
                )
                labels.append(
                    f'<text x="{plot_x0 - 8:.2f}" y="{gy + 3:.2f}" font-size="10" text-anchor="end">{y_val:.2g}</text>'
                )

            polyline = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
            panels.append(f'''
<g transform="translate(0 0)">
<text x="{x0 + panel_w / 2:.2f}" y="{y0 + 15:.2f}" font-size="15" text-anchor="middle">{html.escape(panel_title)}</text>
{''.join(grid)}
<line x1="{plot_x0:.2f}" y1="{plot_y0 + plot_h:.2f}" x2="{plot_x0 + plot_w:.2f}" y2="{plot_y0 + plot_h:.2f}" stroke="#333" stroke-width="1"/>
<line x1="{plot_x0:.2f}" y1="{plot_y0:.2f}" x2="{plot_x0:.2f}" y2="{plot_y0 + plot_h:.2f}" stroke="#333" stroke-width="1"/>
<polyline fill="none" stroke="#1f77b4" stroke-width="1.7" points="{polyline}"/>
{''.join(labels)}
<text x="{plot_x0 + plot_w / 2:.2f}" y="{y0 + panel_h - 5:.2f}" font-size="11" text-anchor="middle">Time [s]</text>
<text transform="translate({x0 + 13:.2f} {plot_y0 + plot_h / 2:.2f}) rotate(-90)" font-size="11" text-anchor="middle">{html.escape(y_label)}</text>
</g>''')

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<g font-family="Arial, sans-serif" fill="#222">
<text x="{width / 2}" y="36" font-size="28" text-anchor="middle">{html.escape(title)}</text>
{''.join(panels)}
</g>
</svg>
'''
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(svg)

    def run(self):
        """Start IMU thread, calibrate, then enter 500 Hz control loop."""
        print("=" * 60)
        print("  SrOmBURo PID Torque Controller")
        print("  Axes: roll=lateral  pitch=longitudinal")
        print("=" * 60)

        # Start IMU background thread
        self._imu_stop.clear()
        self._imu_thread = threading.Thread(
            target=_imu_reader_thread,
            args=(self.imu, self.cfg, self._imu_stop),
            daemon=True,
        )
        self._imu_thread.start()

        print("[Init] Waiting for first IMU packet...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond in 8 s — check cable")
            self._stop()
            return

        # Switch to torque/current control mode
        self._set_torque_mode()

        # Calibrate static offsets
        self._calibrate()

        self._wait_for_space_start()

        #Turns on torque for motors
        self.robot.toggleTorque(1)
        time.sleep(0.2)

        #Sets motor to torque mode (0)
        self.robot.setTorqueMode()
        self._command_ramp_t0 = time.perf_counter()

        print(f"\n[Init] Loop rate : {self.cfg.CTRL_HZ} Hz")
        print(f"[Init] KP_ROLL={self.cfg.KP_ROLL}  KD_ROLL={self.cfg.KD_ROLL}  "
              f"KI_ROLL={self.cfg.KI_ROLL}")
        print(f"[Init] KP_PITCH={self.cfg.KP_PITCH}  KD_PITCH={self.cfg.KD_PITCH}  "
              f"KI_PITCH={self.cfg.KI_PITCH}")
        print(f"[Init] MAX_TORQUE={self.cfg.MAX_TORQUE} Nm  "
              f"FALL_STOP={self.cfg.FALL_DEG}°")
        print(f"[Init] COMMAND_RAMP={self.cfg.COMMAND_RAMP_SEC:.1f} s")
        if self._log_path is not None:
            print(f"[Init] CSV debug log: {self._log_path}")
        print("[Init] Balancing active — Ctrl+C to stop\n")

        self._running = True
        self._control_loop()

    def _wait_for_space_start(self):
        """Block after calibration until SPACE is pressed in the terminal."""
        print("\n[Init] Calibration complete.")
        print("[Init] Press SPACE to enable torque and start the 3 s command ramp...")

        if not sys.stdin.isatty():
            input("[Init] stdin is not a TTY; press Enter to start instead...")
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == " ":
                    print("[Init] SPACE received — enabling torque.")
                    return
                if ch == "\x03":
                    raise KeyboardInterrupt
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _command_ramp_scale(self) -> float:
        """Return 0.0 to 1.0 linear command scale after SPACE start."""
        if self._command_ramp_t0 is None:
            return 0.0
        ramp_sec = max(self.cfg.COMMAND_RAMP_SEC, 1e-9)
        elapsed = time.perf_counter() - self._command_ramp_t0
        return max(0.0, min(elapsed / ramp_sec, 1.0))

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

        # ── 1. Sensor → body angles (control2_Suraj.py convention) ──────────
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
            if vel_w is None:
                vel_w = self._last_vel_w
            else:
                self._last_vel_w = vel_w

            if vel_r is None:
                vel_r = self._last_vel_r
            else:
                self._last_vel_r = vel_r
        except Exception:
            vel_w = self._last_vel_w
            vel_r = self._last_vel_r

        # Motor wiring → axis velocities, matching control2_Suraj.py.
        #   id1 roller    → roll axis
        #   id2 big wheel → pitch axis
        vel_roll_raw  = vel_r
        vel_pitch_raw = vel_w

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

        # ── 5. PID + Feedforward 토크 계산 ───────────────────────────────────────
        # 5.1 Feedback (PID)
        tau_fb_roll  = self.pid_roll.compute(roll_f,  rolldot_f,  vel_roll_f)
        tau_fb_pitch = self.pid_pitch.compute(pitch_f, pitchdot_f, vel_pitch_f)

        # 5.2 Dynamic Gravity Feedforward: quasi-static EOM
        #   m_b L cos(theta) x_ddot - m_b g L sin(theta) = -tau
        #   M_eff x_ddot = tau / r
        # => tau_hold = m_b g L sin(theta) / (1 + m_b L cos(theta)/(r M_eff))
        # Positive tilt needs negative internal torque; roll is inverted later by
        # tau_roll_total, so this sign follows the existing PID convention.
        denom_roll = 1.0 + (
            cfg.M_BODY * cfg.L_COM_ROLL * math.cos(roll_f) /
            (cfg.R_DRIVE_ROLL * cfg.M_EFF_ROLL)
        )
        denom_pitch = 1.0 + (
            cfg.M_BODY * cfg.L_COM_PITCH * math.cos(pitch_f) /
            (cfg.R_DRIVE_PITCH * cfg.M_EFF_PITCH)
        )
        tau_ff_roll_g = -(cfg.MGL_BODY_ROLL * math.sin(roll_f)) / denom_roll
        tau_ff_pitch_g = -(cfg.MGL_BODY_PITCH * math.sin(pitch_f)) / denom_pitch

        # 5.3 Friction Feedforward (tanh — sign 대신 연속 함수로 채터링 방지)
        tau_ff_roll_f  =  -cfg.FRIC_ROLL  * math.tanh(vel_roll_f  * cfg.FRIC_TANH_SCALE)
        tau_ff_pitch_f =  -cfg.FRIC_PITCH * math.tanh(pitchdot_f * cfg.FRIC_TANH_SCALE)

        tau_ff_roll  = tau_ff_roll_g  + tau_ff_roll_f
        tau_ff_pitch = tau_ff_pitch_g + tau_ff_pitch_f

        # Total = Feedback + Feedforward.
        # Roll is inverted to match control2_Suraj.py's positive roll_cmd direction.
        tau_roll_total  = -(tau_fb_roll  + tau_ff_roll)
        tau_pitch_total = tau_fb_pitch + tau_ff_pitch

        # ── 6. Motor output, matching control2_Suraj.py ──────────────────────
        tau_motor2_raw = tau_pitch_total + tau_roll_total/4
        tau_motor1_raw = tau_roll_total

        # Saturate
        tau_motor2_sat = max(-cfg.MAX_TORQUE,
                             min(tau_motor2_raw, cfg.MAX_TORQUE))
        tau_motor1_sat = max(-cfg.MAX_TORQUE,
                             min(tau_motor1_raw, cfg.MAX_TORQUE))

        # Soft-start: after SPACE, ramp actual motor commands from 0% to 100%.
        ramp_scale = self._command_ramp_scale()
        tau_motor2_ramped = tau_motor2_sat * ramp_scale
        tau_motor1_ramped = tau_motor1_sat * ramp_scale

        tau_motor2 = tau_motor2_ramped
        tau_motor1 = tau_motor1_ramped

        # Below noise floor → send zero (don't waste current on micro-commands)
        if abs(tau_motor2) < cfg.MIN_TORQUE: tau_motor2 = 0.0
        if abs(tau_motor1) < cfg.MIN_TORQUE: tau_motor1 = 0.0

        # ── 7. Send to motors ─────────────────────────────────────────────────
        # Omburo.setTorque(id_wheel=2, id_roller=1)
        self.robot.setTorque(tau_motor2, tau_motor1)
        

        # ── 8. Debug print (~1 Hz) ────────────────────────────────────────────
        if tick % cfg.PRINT_EVERY == 0:
            if self._log_writer is not None:
                self._log_writer.writerow([
                    tick,
                    time.perf_counter() - self._log_t0,
                    euler[cfg.ROLL_EU_IDX],
                    euler[cfg.PITCH_EU_IDX],
                    gyro[cfg.ROLLDOT_IDX],
                    gyro[cfg.PITCHDOT_IDX],
                    math.degrees(self._roll_offset),
                    math.degrees(self._pitch_offset),
                    roll,
                    pitch,
                    rolldot,
                    pitchdot,
                    vel_roll_raw,
                    vel_pitch_raw,
                    roll_f,
                    pitch_f,
                    math.degrees(roll_f),
                    math.degrees(pitch_f),
                    rolldot_f,
                    pitchdot_f,
                    vel_roll_f,
                    vel_pitch_f,
                    tau_fb_roll,
                    tau_fb_pitch,
                    denom_roll,
                    denom_pitch,
                    tau_ff_roll_g,
                    tau_ff_pitch_g,
                    tau_ff_roll_f,
                    tau_ff_pitch_f,
                    tau_ff_roll,
                    tau_ff_pitch,
                    tau_roll_total,
                    tau_pitch_total,
                    tau_motor1_raw,
                    tau_motor1_sat,
                    tau_motor1_ramped,
                    tau_motor1,
                    tau_motor2_raw,
                    tau_motor2_sat,
                    tau_motor2_ramped,
                    tau_motor2,
                    ramp_scale,
                ])
                self._log_file.flush()

            print(
                f"\n"
                f"  RAW ROLL    : euler={euler[cfg.ROLL_EU_IDX]:+7.3f}°  "
                f"gyro={gyro[cfg.ROLLDOT_IDX]:+8.4f} rad/s  "
                f"offset={math.degrees(self._roll_offset):+7.3f}°\n"
                f"  ROLL  (lat) : angle={math.degrees(roll_f):+6.2f}°  "
                f"rate={math.degrees(rolldot_f):+6.2f}°/s  "
                f"vel={vel_roll_f:+.3f} rad/s  "
                f"τ_total={tau_roll_total:+.4f} Nm\n"
                f"      PID={tau_fb_roll:+.4f}  "
                f"FF={tau_ff_roll:+.4f} "
                f"(grav={tau_ff_roll_g:+.4f}, fric={tau_ff_roll_f:+.4f}) Nm  "
                f"denom={denom_roll:+.4f}\n"
                f"  PITCH (long): angle={math.degrees(pitch_f):+6.2f}°  "
                f"rate={math.degrees(pitchdot_f):+6.2f}°/s  "
                f"vel={vel_pitch_f:+.3f} rad/s  "
                f"τ_total={tau_pitch_total:+.4f} Nm\n"
                f"      PID={tau_fb_pitch:+.4f}  "
                f"FF={tau_ff_pitch:+.4f} "
                f"(grav={tau_ff_pitch_g:+.4f}, fric={tau_ff_pitch_f:+.4f}) Nm\n"
                f"  ROLL MOTOR  : id1 raw={tau_motor1_raw:+.4f}  "
                f"sat={tau_motor1_sat:+.4f}  "
                f"ramped={tau_motor1_ramped:+.4f}  "
                f"final={tau_motor1:+.4f} Nm\n"
                f"  WHEEL MIX   : id2 raw={tau_motor2_raw:+.4f}  "
                f"sat={tau_motor2_sat:+.4f}  "
                f"ramped={tau_motor2_ramped:+.4f}  "
                f"final={tau_motor2:+.4f} Nm\n"
                f"  MOTORS      : id2(wheel)={tau_motor2:+.4f} Nm  "
                f"id1(roller)={tau_motor1:+.4f} Nm  "
                f"ramp={ramp_scale * 100.0:5.1f}%",
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
        self._imu_stop.set()
        if self._imu_thread is not None and self._imu_thread.is_alive():
            self._imu_thread.join(timeout=1.0)
        if self._log_file is not None:
            try:
                self._log_file.close()
                print(f"[Controller] CSV log saved: {self._log_path}")
                self._plot_debug_csv()
            except Exception:
                pass
            self._log_file = None
            self._log_writer = None
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
    try:
        ctrl.run()
    except KeyboardInterrupt:
        print("\n[Controller] Interrupted during startup...")
        ctrl._stop()