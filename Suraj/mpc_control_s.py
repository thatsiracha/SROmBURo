"""
SrOmBURo balance controller — Sliding Mode Control with BEAR velocity actuation.

Outer loop: SMC computes axis torque [Nm] from IMU state.
Actuation: torque → wheel/roller velocity [rad/s] via OmburoVel (mode 1).

Run:
    python3 mpc_control_s.py
    python3 mpc_control_s.py --lambda 10 --k-switch 4
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
import serial

sys.path.insert(0, "/home/omburo/Documents/SROmBURo/Suraj")
from OmburoVel import OmburoVel

IMU_PORT = "/dev/ttyACM0"
IMU_BAUD = 115200
IMU_RATE_HZ = 200

M_BODY = 4.0
L_COM = 0.5
I_BODY = M_BODY * L_COM ** 2 / 3.0
M_WHEEL = 1.0
R_WHEEL = 0.101
I_WHEEL = 0.5 * M_WHEEL * R_WHEEL ** 2
N_ROLLER = 4.0
G = 9.81

ROLL_EU_IDX = 0
ROLL_EU_SIGN = 1.0
PITCH_EU_IDX = 1
PITCH_EU_SIGN = -1.0
ROLLDOT_IDX = 0
ROLLDOT_SIGN = 1.0
PITCHDOT_IDX = 1
PITCHDOT_SIGN = -1.0
PHI_POS_SIGN_W = -1.0
PHI_POS_SIGN_R = 1.0

FALL_DEG = 40.0
CTRL_HZ = 500
CTRL_DT = 1.0 / CTRL_HZ
VEL_MAX = 50.0          # motor velocity saturation [rad/s]
TAU_TO_VEL = 8.0        # SMC torque [Nm] → velocity [rad/s]
SOFTSTART_S = 0.5

EMA_ALPHA_ANG = 0.25
EMA_ALPHA_RATE = 0.35
EMA_ALPHA_PHI = 0.6

SMC_LAMBDA = 12.0
SMC_K_SWITCH = 8.0
SMC_PHI = 0.08
SMC_USE_WHEEL_STATE = False


def build_axis_model(gear_ratio: float = 1.0, motor_sign: float = 1.0):
    m_b, l, m_w, R, I_b, I_w = M_BODY, L_COM, M_WHEEL, R_WHEEL, I_BODY, I_WHEEL
    M_mat = np.array([
        [(m_b + m_w) * R ** 2 + I_w, m_b * R * l],
        [m_b * R * l, m_b * l ** 2 + I_b],
    ])
    Kg = np.array([[0.0, 0.0], [0.0, -m_b * G * l]])
    Bt = np.array([[motor_sign], [-motor_sign]])
    Minv = np.linalg.inv(M_mat)
    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2:4, 0:2] = -Minv @ Kg
    B = np.zeros((4, 1))
    B[2:4, 0:1] = Minv @ Bt * gear_ratio
    return A, B


@dataclass
class AxisPlant:
    name: str
    a21: float
    a22: float
    b21: float


def _reduced_tilt_coeffs(A: np.ndarray, B: np.ndarray) -> tuple[float, float, float]:
    return float(A[3, 1]), float(A[3, 3]), float(B[3, 0])


def make_plants() -> tuple[AxisPlant, AxisPlant]:
    A_p, B_p = build_axis_model(gear_ratio=1.0, motor_sign=-1.0)
    A_r, B_r = build_axis_model(gear_ratio=N_ROLLER, motor_sign=1.0)
    pitch = AxisPlant("pitch", *_reduced_tilt_coeffs(A_p, B_p))
    roll = AxisPlant("roll", *_reduced_tilt_coeffs(A_r, B_r))
    return pitch, roll


class AxisSMC:
    def __init__(self, plant: AxisPlant, tau_max: float):
        self.plant = plant
        self.lambda_s = SMC_LAMBDA
        self.k_sw = SMC_K_SWITCH
        self.phi = SMC_PHI
        self.tau_max = tau_max

    def compute(self, theta: float, theta_dot: float, phi: float = 0.0, phi_dot: float = 0.0) -> float:
        p = self.plant
        s = theta_dot + self.lambda_s * theta
        sat_s = (1.0 if s > 0.0 else -1.0) if abs(s) > self.phi else s / self.phi
        f_x = p.a21 * theta + p.a22 * theta_dot
        tau_eq = (-f_x - self.lambda_s * theta_dot) / p.b21
        tau_sw = (-self.k_sw * sat_s) / p.b21
        u = tau_eq + tau_sw
        if SMC_USE_WHEEL_STATE:
            u += -(0.05 * phi + 0.02 * phi_dot)
        return float(np.clip(u, -self.tau_max, self.tau_max))


# ── MIP IMU (same protocol as control2_Suraj.py) ──────────────────────────────
MIP_SYNC1 = 0x75
MIP_SYNC2 = 0x65
DESC_BASE = 0x01
DESC_3DM = 0x0C
DESC_IMU = 0x80
FIELD_GYRO = 0x05
FIELD_EULER = 0x0C
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
    header = bytes([MIP_SYNC1, MIP_SYNC2, desc_set, len(payload)])
    body = header + payload
    cs1, cs2 = _fletcher(body)
    return body + bytes([cs1, cs2])


def _cmd_ping():
    return _build_packet(DESC_BASE, [_build_field(0x01)])


def _cmd_imu_format(rate_hz, field_list):
    dec = max(1, MIP_BASE_RATE // rate_hz)
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
                buf = buf[-1:]
                break
            buf = buf[idx:]
            if len(buf) < 4:
                break
            plen = buf[3]
            total = 4 + plen + 2
            if len(buf) < total:
                break
            pkt = buf[:total]
            if _fletcher(pkt[:-2]) != (pkt[-2], pkt[-1]):
                buf = buf[1:]
                continue
            buf = buf[total:]
            return pkt[2], pkt[4:4 + plen]
    return None


def _imu_ack(ser, pkt, cmd_desc, retries=3):
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
            if len(pl) >= 4 and pl[1] == 0xF1 and pl[2] == cmd_desc and pl[3] == 0x00:
                return True
    return False


def _parse_imu_payload(payload):
    euler = gyro = None
    i = 0
    while i + 1 < len(payload):
        flen = payload[i]
        fdesc = payload[i + 1]
        fdata = payload[i + 2:i + flen]
        if fdesc == FIELD_EULER and len(fdata) >= 12:
            roll, pitch, yaw = struct.unpack(">fff", fdata[:12])
            euler = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        elif fdesc == FIELD_GYRO and len(fdata) >= 12:
            gyro = struct.unpack(">fff", fdata[:12])
        i += max(flen, 1)
    return euler, gyro


class IMUState:
    def __init__(self):
        self._lock = threading.Lock()
        self.euler = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.updated = threading.Event()

    def push(self, euler=None, gyro=None):
        with self._lock:
            if euler is not None:
                self.euler = euler
            if gyro is not None:
                self.gyro = gyro
        self.updated.set()

    def get(self):
        with self._lock:
            return self.euler, self.gyro


def _imu_thread(state: IMUState):
    try:
        with serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.05, dsrdtr=False, rtscts=False) as ser:
            time.sleep(1.5)
            ser.reset_input_buffer()
            if not _imu_ack(ser, _cmd_ping(), 0x01):
                print("[IMU] no ping response")
                return
            _imu_ack(ser, _cmd_imu_stream(False), 0x11)
            time.sleep(0.1)
            ser.reset_input_buffer()
            if not _imu_ack(ser, _cmd_imu_format(IMU_RATE_HZ, [FIELD_EULER, FIELD_GYRO]), 0x08):
                print("[IMU] format setup failed")
                return
            if not _imu_ack(ser, _cmd_imu_stream(True), 0x11):
                print("[IMU] stream enable failed")
                return
            print("[IMU] stream active")
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
        print(f"[IMU] thread error: {exc}")


class BalanceController:
    def __init__(self, tau_max: float):
        self.imu = IMUState()
        self.robot = OmburoVel()
        self.pitch_plant, self.roll_plant = make_plants()
        self.pitch_ctrl = AxisSMC(self.pitch_plant, tau_max)
        self.roll_ctrl = AxisSMC(self.roll_plant, tau_max)

        self._roll_offset = self._pitch_offset = 0.0
        self._phi_w_zero = self._phi_r_zero = 0.0
        self._roll_f = self._pitch_f = 0.0
        self._rolldot_f = self._pitchdot_f = 0.0
        self._phi_w_f = self._phi_r_f = 0.0
        self._step_count = 0
        self._softstart_steps = int(SOFTSTART_S * CTRL_HZ)
        self._print_model_summary(tau_max)

    def _print_model_summary(self, tau_max: float) -> None:
        pp, rp = self.pitch_plant, self.roll_plant
        print(f"[model] m={M_BODY} kg  L_com={L_COM} m  tau_max={tau_max:.2f} Nm  vel_max={VEL_MAX} rad/s")
        print(f"[model] pitch θ̈: a21={pp.a21:.2f}  a22={pp.a22:.2f}  b21={pp.b21:.3f}")
        print(f"[model] roll  θ̈: a21={rp.a21:.2f}  a22={rp.a22:.2f}  b21={rp.b21:.3f}")
        wn = math.sqrt(abs(pp.a21))
        print(f"[model] ω_n ≈ {wn:.2f} rad/s")

    def _calibrate(self, n: int = 50) -> None:
        roll_sum = pitch_sum = 0.0
        print(f"Calibrating IMU ({n} samples) — hold robot upright and still...")
        for _ in range(n):
            self.imu.updated.wait()
            self.imu.updated.clear()
            euler, _ = self.imu.get()
            roll_sum += ROLL_EU_SIGN * math.radians(euler[ROLL_EU_IDX])
            pitch_sum += PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX])
        self._roll_offset = roll_sum / n
        self._pitch_offset = pitch_sum / n
        pos_w, _, pos_r, _ = self.robot.readback()
        self._phi_w_zero = pos_w
        self._phi_r_zero = pos_r
        print(f"  offsets: roll={math.degrees(self._roll_offset):+.2f}°  "
              f"pitch={math.degrees(self._pitch_offset):+.2f}°")

    def run(self) -> None:
        threading.Thread(target=_imu_thread, args=(self.imu,), daemon=True).start()
        print("Waiting for IMU...")
        if not self.imu.updated.wait(timeout=8.0):
            print("[ERROR] IMU timeout")
            self.robot.close()
            return

        self._calibrate()
        self.robot.setVelocityMode()
        self.robot.setVelocity(0.0, 0.0)
        self.robot.toggleTorque(1)
        print("Balancing (SMC, velocity mode) — Ctrl+C to stop\n")

        t_next = time.perf_counter()
        try:
            while True:
                now = time.perf_counter()
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
            self.robot.setVelocity(0.0, 0.0)
            self.robot.toggleTorque(0)
            self.robot.close()

    def _torque_to_velocity(self, tau_pitch: float, tau_roll: float) -> tuple[float, float]:
        """Map SMC torque to motor velocities (matches control2_Suraj wiring/signs)."""
        pitch_cmd = -tau_pitch * N_ROLLER * TAU_TO_VEL
        roll_cmd = tau_roll * TAU_TO_VEL
        vel_wheel = float(np.clip(pitch_cmd, -VEL_MAX, VEL_MAX))
        vel_roller = float(np.clip(roll_cmd, -VEL_MAX, VEL_MAX))
        return vel_wheel, vel_roller

    def _step(self) -> None:
        self._step_count += 1
        euler, gyro = self.imu.get()

        roll = ROLL_EU_SIGN * math.radians(euler[ROLL_EU_IDX]) - self._roll_offset
        pitch = PITCH_EU_SIGN * math.radians(euler[PITCH_EU_IDX]) - self._pitch_offset
        rolldot = ROLLDOT_SIGN * gyro[ROLLDOT_IDX]
        pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]

        if abs(roll) > math.radians(FALL_DEG) or abs(pitch) > math.radians(FALL_DEG):
            self.robot.setVelocity(0.0, 0.0)
            return

        try:
            pos_w, vel_w, pos_r, vel_r = self.robot.readback()
        except Exception:
            pos_w = vel_w = pos_r = vel_r = 0.0

        phi_w = PHI_POS_SIGN_W * (pos_w - self._phi_w_zero)
        phi_r = PHI_POS_SIGN_R * (pos_r - self._phi_r_zero)

        a, ar, ap = EMA_ALPHA_ANG, EMA_ALPHA_RATE, EMA_ALPHA_PHI
        self._roll_f = a * self._roll_f + (1 - a) * roll
        self._pitch_f = a * self._pitch_f + (1 - a) * pitch
        self._rolldot_f = ar * self._rolldot_f + (1 - ar) * rolldot
        self._pitchdot_f = ar * self._pitchdot_f + (1 - ar) * pitchdot
        self._phi_w_f = ap * self._phi_w_f + (1 - ap) * vel_w
        self._phi_r_f = ap * self._phi_r_f + (1 - ap) * vel_r

        roll, pitch = self._roll_f, self._pitch_f
        rolldot, pitchdot = self._rolldot_f, self._pitchdot_f

        if SMC_USE_WHEEL_STATE:
            phi_w_s, phi_r_s = phi_w, N_ROLLER * phi_r
            phi_w_ds, phi_r_ds = self._phi_w_f, N_ROLLER * self._phi_r_f
        else:
            phi_w_s = phi_r_s = phi_w_ds = phi_r_ds = 0.0

        tau_pitch = self.pitch_ctrl.compute(pitch, pitchdot, phi_w_s, phi_w_ds)
        tau_roll = self.roll_ctrl.compute(roll, rolldot, phi_r_s, phi_r_ds)

        ramp = min(1.0, self._step_count / max(1, self._softstart_steps))
        tau_pitch *= ramp
        tau_roll *= ramp

        vel_wheel, vel_roller = self._torque_to_velocity(tau_pitch, tau_roll)
        self.robot.setVelocity(vel_wheel, vel_roller)

        if self._step_count % 50 == 0:
            print(
                f"\r[smc] roll={math.degrees(roll):+5.2f}° pitch={math.degrees(pitch):+5.2f}°  "
                f"τ_p={tau_pitch:+.2f} τ_r={tau_roll:+.2f} Nm  "
                f"v_p={vel_wheel:+.1f} v_r={vel_roller:+.1f} rad/s",
                end="",
                flush=True,
            )


def main():
    global SMC_LAMBDA, SMC_K_SWITCH, TAU_TO_VEL

    from OmburoVel import kt, iq as IQ_HW_LIMIT

    parser = argparse.ArgumentParser(description="SrOmBURo SMC balance (velocity mode)")
    parser.add_argument("--lambda", dest="lambda_s", type=float, default=SMC_LAMBDA)
    parser.add_argument("--k-switch", type=float, default=SMC_K_SWITCH)
    parser.add_argument("--tau-max", type=float, default=None,
                        help="SMC torque saturation [Nm]; default kt*iq")
    parser.add_argument("--tau-to-vel", type=float, default=TAU_TO_VEL,
                        help="Torque-to-velocity scale [rad/s per Nm]")
    args = parser.parse_args()

    SMC_LAMBDA = args.lambda_s
    SMC_K_SWITCH = args.k_switch
    TAU_TO_VEL = args.tau_to_vel
    tau_max = args.tau_max if args.tau_max is not None else kt * IQ_HW_LIMIT

    BalanceController(tau_max=tau_max).run()


if __name__ == "__main__":
    main()
