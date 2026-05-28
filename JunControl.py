"""
OmBURo LQR-PI balancing controller for Raspberry Pi

Dual-axis wheeled inverted pendulum with:
  - LQR inner loop  (full-state feedback torque control)
  - PI outer loop   (station keeping; compensates IMU bias and CoM offset)

State:   x = [θ1, θ2, θ̇1, θ̇2, φ̇1, φ̇2]
           θ1/θ2   : pitch/roll body angle [rad]
           θ̇1/θ̇2  : angular rates from IMU [rad/s]
           φ̇1/φ̇2  : wheel/roller velocity relative to body [rad/s]

Control: u = [u1 (wheel torque), u2 (roller torque)]  [N·m]

Reference: Shen & Hong, "OmBURo: A Novel Unicycle Robot with Active
           Omnidirectional Wheel," arXiv:2001.07856, 2020.
"""

import math
import struct
import sys
import threading
import time

import numpy as np
import scipy.linalg
import serial
from pybear import Manager

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from Omburo import Omburo

# ── Hardware ──────────────────────────────────────────────────────────────────
IMU_PORT     = "/dev/ttyACM0"
IMU_BAUD     = 115200
IMU_RATE_HZ  = 200          # must divide 500 (IMU base rate) evenly

ID_WHEEL     = 2            # BEAR motor: main wheel  (longitudinal)
ID_ROLLER    = 1            # BEAR motor: roller      (lateral)

# Torque control mode for BEAR actuators via pybear.
# Common mapping: 0=disable, 1=velocity, 2=position, 3=torque/current.
# Verify against your firmware version before running.
MOTOR_MODE_TORQUE = 3

# ── Physical parameters (Table II, Shen & Hong 2020) ─────────────────────────
R    = 0.101      # main-wheel radius [m]
r    = 0.0142     # roller radius [m]
l    = 872.523560e-3     # wheel–body COM distance [m]
mw   = 0.72       # wheel mass [kg]
mb   = 4.4491       # body mass [kg]
m    = mb + mw    # total mass [kg]
G    = 9.81       # gravitational acceleration [m/s²]

# Moments of inertia [kg·m²]  (table values in g·m² = 10⁻³ kg·m²)
Iwx  = 1.925e-3
Iwy  = 3.706e-3
Ibx  = 1868.9101e-3
Iby  = 16.9568e-3
Ibz  = 1868.3023e-3
Irx  = 0.030e-3

# Viscous damping [N·m·s/rad]  (table: N·cm/(rad/s) → ×10⁻²)
MU_G = 9.0e-2     # roller–ground contact
MU1  = 0.5e-2     # wheel axle
MU2  = 0.5e-2     # roller axle

# ── IMU axis mapping ──────────────────────────────────────────────────────────
# IMU euler tuple = (roll_deg, pitch_deg, yaw_deg)
# IMU gyro  tuple = (ωx, ωy, ωz) rad/s
# Adjust indices / signs for your physical mounting orientation.
THETA1_EU_IDX    = 1;  THETA1_EU_SIGN    = 1.0   # pitch → θ1 (longitudinal)
THETA2_EU_IDX    = 0;  THETA2_EU_SIGN    = -1.0   # roll  → θ2 (lateral)
DTHETA1_GY_IDX   = 1;  DTHETA1_GY_SIGN   = 1.0   # ωy    → θ̇1
DTHETA2_GY_IDX   = 0;  DTHETA2_GY_SIGN   = -1.0   # ωx    → θ̇2

# ── Safety & timing ───────────────────────────────────────────────────────────
FALL_DEG     = 30.0    # body angle threshold triggering fall-stop [deg]
TORQUE_MAX   = 2.5     # motor torque saturation [N·m]
CTRL_HZ      = 200
CTRL_DT      = 1.0 / CTRL_HZ

# ── LQR tuning ────────────────────────────────────────────────────────────────
# State penalty Q = diag[θ1, θ2, θ̇1, θ̇2, φ̇1, φ̇2]
# Higher angle weights → stiffer balance; tune on hardware starting here.
Q_DIAG = [600.0, 600.0, 80.0, 80.0, 1.0, 1.0]
R_DIAG = [0.05,  0.05]          # control-effort penalty

# ── PI outer-loop (station keeping) ──────────────────────────────────────────
PI_KP  = 5 #0.3
PI_KI  = 0.1
PI_LIM = 0.5    # max PI velocity output [rad/s]

# ── State measurement filter ──────────────────────────────────────────────────
EMA_ALPHA = 0.7   # exponential moving average coefficient for angle/rate


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
        self.euler   = (0.0, 0.0, 0.0)   # (roll_deg, pitch_deg, yaw_deg)
        self.gyro    = (0.0, 0.0, 0.0)   # (ωx, ωy, ωz) [rad/s]
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


# ─── PI controller ────────────────────────────────────────────────────────────
class PIController:
    def __init__(self, kp, ki, limit):
        self.kp = kp; self.ki = ki; self.limit = limit
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def update(self, error, dt):
        max_integral = self.limit / max(self.ki, 1e-9)
        self._integral = np.clip(self._integral + error * dt,
                                 -max_integral, max_integral)
        return float(np.clip(self.kp * error + self.ki * self._integral,
                             -self.limit, self.limit))


# ─── System matrices (Appendix B, Shen & Hong 2020) ──────────────────────────
def _build_system():
    """Linearised OmBURo state-space A (6×6) and B (6×2) at upright equilibrium."""
    M33 = Iwy + m * R**2
    M44 = Irx + m * r**2

    a1 = Iby + Iwy + mb*l**2 + m*R**2 + 2*mb*l*R
    a2 = Iwy + m*R**2 + mb*l*R
    a3 = -mb * G * l
    a4 = Ibx + Irx + Iwx + mb*l**2 + m*R**2 + 2*mb*l*R
    a5 = Irx + m*r*R + mb*l*r
    a6 = -mb*G*l - m*G*(R - r)

    d1 = a2**2 - a1*M33
    d2 = a5**2 - a4*M44

    A = np.array([
        [0,          0,          1,                        0,                        0,                              0                            ],
        [0,          0,          0,                        1,                        0,                              0                            ],
        [a3*M33/d1,  0,          MU_G*(M33-a2)/d1,         0,           (MU_G*M33-(MU_G+MU1)*a2)/d1,               0                            ],
        [0,          a6*M44/d2,  0,                        MU_G*(M44-a5)/d2,         0,               (MU_G*M44-(MU_G+MU2)*a5)/d2               ],
        [-a2*a3/d1,  0,          MU_G*(a1-a2)/d1,          0,           (-MU_G*a2+(MU_G+MU1)*a1)/d1,               0                            ],
        [0,          -a5*a6/d2,  0,                        MU_G*(a4-a5)/d2,          0,               (-MU_G*a5+(MU_G+MU2)*a4)/d2               ],
    ], dtype=float)

    B = np.array([
        [0,       0      ],
        [0,       0      ],
        [a2/d1,   0      ],
        [0,       a5/d2  ],
        [-a1/d1,  0      ],
        [0,       -a4/d2 ],
    ], dtype=float)

    return A, B


def _solve_lqr(A, B, Q, R):
    """Continuous-time LQR: return gain K (2×6) and feedforward H (2×2)."""
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)

    # H chosen so that steady-state ψ̇ tracks ψ̇_ref under constant reference
    C = np.array([[0, 0, 1, 0, 1, 0],
                  [0, 0, 0, 1, 0, 1]], dtype=float)
    H = np.linalg.inv(C @ np.linalg.solve(B @ K - A, B))
    return K, H


# ─── Main controller ──────────────────────────────────────────────────────────
class Controller:
    def __init__(self):
        A, B = _build_system()
        Q = np.diag(Q_DIAG)
        R = np.diag(R_DIAG)
        self.K, self.H = _solve_lqr(A, B, Q, R)
        print("LQR gain K:\n", self.K)

        self.pi1 = PIController(PI_KP, PI_KI, PI_LIM)   # longitudinal (wheel)
        self.pi2 = PIController(PI_KP, PI_KI, PI_LIM)   # lateral (roller)

        self.imu   = IMUState()
        self.robot = Omburo()

        # Switch motors to torque (current) control for direct LQR force output.
        # Comment out if set_goal_iq works correctly in your firmware's velocity mode.
        self.robot.bear.set_mode(
            (ID_WHEEL,  MOTOR_MODE_TORQUE),
            (ID_ROLLER, MOTOR_MODE_TORQUE),
        )

        self._vel_ref   = np.zeros(2)   # reference wheel/roller velocity [rad/s]
        self._pos_ref   = None          # latched position reference [rad]
        self._x_filt    = np.zeros(6)   # EMA-filtered state

    # ── Public interface ──────────────────────────────────────────────────────
    def set_velocity_reference(self, vx_ms: float, vy_ms: float):
        """Convert desired Cartesian velocity [m/s] to motor velocity reference."""
        # Kinematics: vx = ω1·R,  vy = n·(ω1+ω2)·r   (Eq. 1, paper)
        # n = 4 (inverse gear ratio for roller drive via helical gears)
        n = 4.0
        omega1 = vx_ms / R
        omega2 = vy_ms / (n * r) - omega1
        self._vel_ref = np.array([omega1, omega2])

    def run(self):
        t = threading.Thread(target=_imu_thread, args=(self.imu,), daemon=True)
        t.start()

        print("Waiting for IMU data...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU did not respond — aborting")
            self.robot.close()
            return

        self.robot.toggleTorque(1)
        print("Balancing active — Ctrl+C to stop\n")

        t_next = time.perf_counter()
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
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.robot.setTorque(0.0, 0.0)
            self.robot.toggleTorque(0)
            self.robot.close()

    # ── Control step (runs at CTRL_HZ) ────────────────────────────────────────
    def _step(self):
        euler, gyro = self.imu.get()

        # Map IMU measurements to robot body angles and rates
        theta1  = THETA1_EU_SIGN  * math.radians(euler[THETA1_EU_IDX])
        theta2  = THETA2_EU_SIGN  * math.radians(euler[THETA2_EU_IDX])
        dtheta1 = DTHETA1_GY_SIGN * gyro[DTHETA1_GY_IDX]
        dtheta2 = DTHETA2_GY_SIGN * gyro[DTHETA2_GY_IDX]

        # Fall-stop: disable torque if body exceeds safe angle range
        if (abs(theta1) > math.radians(FALL_DEG) or
                abs(theta2) > math.radians(FALL_DEG)):
            self.robot.setTorque(0.0, 0.0)
            return

        # Read motor shaft position and velocity from encoders
        pos_w, vel_w, pos_r, vel_r = self.robot.readback()

        # Latch position reference on first valid step (station keeping)
        if self._pos_ref is None:
            self._pos_ref = (pos_w, pos_r)
            self.pi1.reset(); self.pi2.reset()

        # Relative velocities: φ̇ = ψ̇ − θ̇
        dphi1 = vel_w - dtheta1
        dphi2 = vel_r - dtheta2

        # Assemble state and apply EMA low-pass filter to reduce sensor noise
        x_raw = np.array([theta1, theta2, dtheta1, dtheta2, dphi1, dphi2])
        self._x_filt = EMA_ALPHA * self._x_filt + (1.0 - EMA_ALPHA) * x_raw

        # Outer PI: velocity correction from position error (station keeping)
        pos_err = np.array([self._pos_ref[0] - pos_w,
                             self._pos_ref[1] - pos_r])
        pi_correction = np.array([self.pi1.update(pos_err[0], CTRL_DT),
                                   self.pi2.update(pos_err[1], CTRL_DT)])
        psi_dot_ref = self._vel_ref + pi_correction

        # Inner LQR: u = −K·x + H·ψ̇_ref
        u = -self.K @ self._x_filt + self.H @ psi_dot_ref

        # Saturate and apply motor torques
        u1 = float(np.clip(u[0], -TORQUE_MAX, TORQUE_MAX))
        u2 = float(np.clip(u[1], -TORQUE_MAX, TORQUE_MAX))
        self.robot.setTorque(u1, u2)


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = Controller()
    # For station keeping only, leave velocity reference at zero (default).
    # For motion, call: ctrl.set_velocity_reference(vx_ms, vy_ms)
    ctrl.run()