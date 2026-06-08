"""
control3.py axis/motor diagnostic

Usage:
  python3 test_control3_axes.py imu
      IMU raw channels and several roll/pitch mappings. No motor commands.

  python3 test_control3_axes.py lqr
      Computes control3 LQR torque commands from IMU only. No motor commands.

  python3 test_control3_axes.py motor --enable
      Sends small torque pulses to each motor separately so you can confirm
      which physical motor is id_wheel and id_roller.

  python3 test_control3_axes.py velocity --enable
      Sends small velocity pulses using the same setVelocity path as control.py
      and control2.py.

  python3 test_control3_axes.py scan
      Pings BEAR IDs 0..10. No motor commands.

  python3 test_control3_axes.py idpulse --enable --ids 1,2,3
      Sends a small velocity pulse to each listed BEAR ID directly, bypassing
      Omburo.py's id_wheel/id_roller mapping.

  python3 test_control3_axes.py monitor --ids 1,2
      Continuously reads position/velocity for listed IDs. No motor commands.
      Move each physical motor by hand to see which ID changes.

  python3 test_control3_axes.py coupling --enable --ids 1,2
      Commands each listed ID one at a time while reading all listed encoder
      positions, so you can see which encoder actually changes.

Suggested order:
  1. Run imu. Tilt only pitch, then only roll. Note which mapping is correct.
  2. Run lqr. Confirm pitch tilt makes wheel torque, roll tilt makes roller torque.
  3. Run motor --enable only with the robot safely lifted/off the ground.
"""

import argparse
import math
import sys
import threading
import time

import numpy as np
from pybear import Manager

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from Omburo import Omburo
from control3 import (
    IMUState,
    _imu_thread,
    ROLL_EU_IDX,
    ROLL_EU_SIGN,
    PITCH_EU_IDX,
    PITCH_EU_SIGN,
    ROLLDOT_IDX,
    ROLLDOT_SIGN,
    PITCHDOT_IDX,
    PITCHDOT_SIGN,
    K_PITCH,
    K_ROLL,
    TORQUE_MIX_MODE,
    KT,
    I_TEST_A,
    I_LIM_A,
)

SAMPLE_HZ = 20
CALIB_SAMPLES = 50
PULSE_TORQUE_NM = 0.12
PULSE_TIME_S = 0.35
PULSE_VELOCITY_RAD_S = 1.0
DEFAULT_SCAN_IDS = range(0, 11)


MAPPINGS = {
    "control_ref": {
        "desc": "control.py/control2.py mapping",
        "roll_idx": 0,
        "roll_sign": 1.0,
        "pitch_idx": 1,
        "pitch_sign": -1.0,
        "rolldot_idx": 0,
        "rolldot_sign": 1.0,
        "pitchdot_idx": 1,
        "pitchdot_sign": -1.0,
    },
    "control3_now": {
        "desc": "current control3.py mapping",
        "roll_idx": ROLL_EU_IDX,
        "roll_sign": ROLL_EU_SIGN,
        "pitch_idx": PITCH_EU_IDX,
        "pitch_sign": PITCH_EU_SIGN,
        "rolldot_idx": ROLLDOT_IDX,
        "rolldot_sign": ROLLDOT_SIGN,
        "pitchdot_idx": PITCHDOT_IDX,
        "pitchdot_sign": PITCHDOT_SIGN,
    },
    "raw_01": {
        "desc": "raw euler[0]=roll, euler[1]=pitch, both positive",
        "roll_idx": 0,
        "roll_sign": 1.0,
        "pitch_idx": 1,
        "pitch_sign": 1.0,
        "rolldot_idx": 0,
        "rolldot_sign": 1.0,
        "pitchdot_idx": 1,
        "pitchdot_sign": 1.0,
    },
    "raw_10": {
        "desc": "raw euler[1]=roll, euler[0]=pitch, both positive",
        "roll_idx": 1,
        "roll_sign": 1.0,
        "pitch_idx": 0,
        "pitch_sign": 1.0,
        "rolldot_idx": 1,
        "rolldot_sign": 1.0,
        "pitchdot_idx": 0,
        "pitchdot_sign": 1.0,
    },
}


def start_imu():
    state = IMUState()
    thread = threading.Thread(target=_imu_thread, args=(state,), daemon=True)
    thread.start()
    print("Waiting for IMU data...", end="", flush=True)
    if not state.updated.wait(timeout=8.0):
        print(" failed")
        sys.exit(1)
    print(" ok\n")
    return state


def mapped_values(euler, gyro, mapping, offsets=None):
    offsets = offsets or (0.0, 0.0)
    roll = mapping["roll_sign"] * math.radians(euler[mapping["roll_idx"]]) - offsets[0]
    pitch = mapping["pitch_sign"] * math.radians(euler[mapping["pitch_idx"]]) - offsets[1]
    rolldot = mapping["rolldot_sign"] * gyro[mapping["rolldot_idx"]]
    pitchdot = mapping["pitchdot_sign"] * gyro[mapping["pitchdot_idx"]]
    return roll, pitch, rolldot, pitchdot


def calibrate_offsets(imu):
    sums = {name: [0.0, 0.0] for name in MAPPINGS}
    print(f"Calibrating {CALIB_SAMPLES} samples. Hold robot still...")
    for _ in range(CALIB_SAMPLES):
        imu.updated.wait()
        imu.updated.clear()
        euler, gyro = imu.get()
        for name, mapping in MAPPINGS.items():
            roll, pitch, _, _ = mapped_values(euler, gyro, mapping)
            sums[name][0] += roll
            sums[name][1] += pitch
    return {
        name: (total[0] / CALIB_SAMPLES, total[1] / CALIB_SAMPLES)
        for name, total in sums.items()
    }


def mode_imu():
    imu = start_imu()
    offsets = calibrate_offsets(imu)
    print("\nTilt test:")
    print("  Physical pitch only: one mapping's pitch should change a lot, roll near 0.")
    print("  Physical roll only : one mapping's roll should change a lot, pitch near 0.")
    print("  Press Ctrl+C to stop.\n")

    try:
        while True:
            euler, gyro = imu.get()
            print("\033[H\033[J", end="")
            print(
                f"raw euler deg: e0={euler[0]:+7.2f}  e1={euler[1]:+7.2f}  e2={euler[2]:+7.2f}"
            )
            print(
                f"raw gyro rad/s: g0={gyro[0]:+7.3f}  g1={gyro[1]:+7.3f}  g2={gyro[2]:+7.3f}\n"
            )
            for name, mapping in MAPPINGS.items():
                roll, pitch, rolldot, pitchdot = mapped_values(
                    euler, gyro, mapping, offsets[name]
                )
                print(
                    f"{name:12s}  roll={math.degrees(roll):+7.2f} deg"
                    f"  pitch={math.degrees(pitch):+7.2f} deg"
                    f"  rdot={rolldot:+7.3f}  pdot={pitchdot:+7.3f}"
                    f"  ({mapping['desc']})"
                )
            time.sleep(1.0 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print()


def mode_lqr():
    imu = start_imu()
    offsets = calibrate_offsets(imu)
    tau_max = KT * min(I_TEST_A, I_LIM_A)
    print("\nLQR dry-run. No motor commands are sent.")
    print("Expected:")
    print("  Physical pitch -> u_pitch/wheel changes, u_roll/roller near 0.")
    print("  Physical roll  -> u_roll/roller changes, u_pitch/wheel near 0.")
    print("  Press Ctrl+C to stop.\n")

    try:
        while True:
            euler, gyro = imu.get()
            print("\033[H\033[J", end="")
            print(
                f"raw euler deg: e0={euler[0]:+7.2f}  e1={euler[1]:+7.2f}"
                f"  | raw gyro: g0={gyro[0]:+7.3f}  g1={gyro[1]:+7.3f}\n"
            )
            for name, mapping in MAPPINGS.items():
                roll, pitch, rolldot, pitchdot = mapped_values(
                    euler, gyro, mapping, offsets[name]
                )
                x_pitch = np.array([0.0, pitch, 0.0, pitchdot])
                x_roll = np.array([0.0, roll, 0.0, rolldot])
                u_pitch = float(np.clip(-float(K_PITCH @ x_pitch), -tau_max, tau_max))
                u_roll = float(np.clip(-float(K_ROLL @ x_roll), -tau_max, tau_max))
                motor_w_ref = float(np.clip(u_pitch, -tau_max, tau_max))
                motor_r_ref = float(np.clip(u_roll - u_pitch, -tau_max, tau_max))
                motor_w_ind = u_pitch
                motor_r_ind = u_roll
                print(
                    f"{name:12s}  angle r/p={math.degrees(roll):+6.2f}/"
                    f"{math.degrees(pitch):+6.2f} deg"
                    f"  u_pitch(wheel)={u_pitch:+7.3f} Nm"
                    f"  u_roll(roller)={u_roll:+7.3f} Nm"
                    f"  ref_motor=({motor_w_ref:+6.3f},{motor_r_ref:+6.3f})"
                    f"  ind_motor=({motor_w_ind:+6.3f},{motor_r_ind:+6.3f})"
                )
            print(f"\ncontrol3 TORQUE_MIX_MODE={TORQUE_MIX_MODE!r}")
            time.sleep(1.0 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print()


def pulse(robot, wheel_tau, roller_tau, label):
    input(f"Press Enter to send {label}: wheel={wheel_tau:+.3f} Nm, roller={roller_tau:+.3f} Nm")
    robot.setTorque(wheel_tau, roller_tau)
    time.sleep(PULSE_TIME_S)
    robot.setTorque(0.0, 0.0)
    time.sleep(0.2)
    try:
        _, vel_w, _, vel_r = robot.readback()
        print(f"  readback after pulse: vel_w={vel_w:+.3f}, vel_r={vel_r:+.3f}\n")
    except Exception as exc:
        print(f"  readback failed: {exc}\n")


def mode_motor(enable):
    if not enable:
        print("Motor pulse test is disabled by default.")
        print("Run: python3 test_control3_axes.py motor --enable")
        print("Lift/support the robot first. This sends real torque commands.")
        return

    robot = Omburo()
    try:
        robot.setTorqueMode()
        robot.setCurrentLimit(0.6)
        robot.setTorque(0.0, 0.0)
        robot.toggleTorque(1)
        print("Motor torque enabled with low current limit.")
        print("Confirm which physical motor moves for each pulse.\n")
        pulse(robot, +PULSE_TORQUE_NM, 0.0, "WHEEL + pulse")
        pulse(robot, -PULSE_TORQUE_NM, 0.0, "WHEEL - pulse")
        pulse(robot, 0.0, +PULSE_TORQUE_NM, "ROLLER + pulse")
        pulse(robot, 0.0, -PULSE_TORQUE_NM, "ROLLER - pulse")
    finally:
        robot.setTorque(0.0, 0.0)
        time.sleep(0.05)
        robot.toggleTorque(0)
        robot.setTorqueMode()
        robot.close()


def velocity_pulse(robot, v_wheel, v_roller_motor, label):
    input(
        f"Press Enter to send {label}: "
        f"setVelocity(wheel={v_wheel:+.2f}, roller_motor={v_roller_motor:+.2f})"
    )
    robot.setVelocity(v_wheel, v_roller_motor)
    time.sleep(PULSE_TIME_S)
    robot.setVelocity(0.0, 0.0)
    time.sleep(0.2)
    try:
        _, vel_w, _, vel_r = robot.readback()
        print(f"  readback after pulse: vel_w={vel_w:+.3f}, vel_r={vel_r:+.3f}\n")
    except Exception as exc:
        print(f"  readback failed: {exc}\n")


def mode_velocity(enable):
    if not enable:
        print("Velocity pulse test is disabled by default.")
        print("Run: python3 test_control3_axes.py velocity --enable")
        print("Lift/support the robot first. This sends real velocity commands.")
        return

    robot = Omburo()
    try:
        robot.setVelocityMode()
        robot.setCurrentLimit(1.5)
        robot.setVelocity(0.0, 0.0)
        robot.toggleTorque(1)
        print("Velocity mode enabled with 1.5 A current limit.")
        print("These pulses test the exact command path used by control.py/control2.py.\n")

        velocity_pulse(robot, +PULSE_VELOCITY_RAD_S, 0.0, "wheel motor +")
        velocity_pulse(robot, -PULSE_VELOCITY_RAD_S, 0.0, "wheel motor -")
        velocity_pulse(robot, 0.0, +PULSE_VELOCITY_RAD_S, "roller motor +")
        velocity_pulse(robot, 0.0, -PULSE_VELOCITY_RAD_S, "roller motor -")

        print("control.py/control2.py mixing examples:")
        velocity_pulse(
            robot,
            0.0,
            +PULSE_VELOCITY_RAD_S,
            "pure roll axis command: setVelocity(0, +roll)",
        )
        velocity_pulse(
            robot,
            +PULSE_VELOCITY_RAD_S,
            -PULSE_VELOCITY_RAD_S,
            "pure pitch axis command: setVelocity(+pitch, -pitch)",
        )
    finally:
        robot.setVelocity(0.0, 0.0)
        time.sleep(0.05)
        robot.toggleTorque(0)
        robot.setTorqueMode()
        robot.close()


def parse_ids(ids_text):
    if ids_text is None:
        return list(DEFAULT_SCAN_IDS)
    ids = []
    for part in ids_text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(part))
    return ids


def open_bear():
    return Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)


def ping_id(bear, motor_id):
    try:
        reply = bear.ping(motor_id)
        data = reply[0][1]
        return data is not None, data
    except Exception as exc:
        return False, f"ERR {exc}"


def mode_scan(ids_text):
    ids = parse_ids(ids_text)
    bear = open_bear()
    try:
        print("Scanning BEAR IDs:", ", ".join(str(i) for i in ids))
        for motor_id in ids:
            ok, data = ping_id(bear, motor_id)
            status = "OK" if ok else "--"
            print(f"  id {motor_id:3d}: {status}  {data}")
    finally:
        bear.close_port()


def direct_velocity_pulse(bear, motor_id, velocity, label):
    input(f"Press Enter for id {motor_id} {label}: velocity={velocity:+.2f} rad/s")
    bear.set_goal_velocity((motor_id, velocity))
    time.sleep(PULSE_TIME_S)
    bear.set_goal_velocity((motor_id, 0.0))
    time.sleep(0.2)
    try:
        vel = bear.get_present_velocity(motor_id)[0][0][0]
        pos = bear.get_present_position(motor_id)[0][0][0]
        print(f"  readback id {motor_id}: pos={pos:+.3f}, vel={vel:+.3f}\n")
    except Exception as exc:
        print(f"  readback failed for id {motor_id}: {exc}\n")


def mode_idpulse(enable, ids_text):
    if not enable:
        print("Direct ID pulse test is disabled by default.")
        print("Run: python3 test_control3_axes.py idpulse --enable --ids 1,2")
        print("Lift/support the robot first. This sends real velocity commands.")
        return

    ids = parse_ids(ids_text)
    bear = open_bear()
    try:
        print("Direct velocity pulses bypass Omburo.py.")
        print("For each responding ID, watch which physical motor moves.\n")
        for motor_id in ids:
            ok, data = ping_id(bear, motor_id)
            if not ok:
                print(f"Skipping id {motor_id}: no ping response ({data})")
                continue
            print(f"id {motor_id} responded: {data}")
            bear.set_mode((motor_id, 1))
            bear.set_limit_i_max((motor_id, 1.5))
            bear.set_goal_velocity((motor_id, 0.0))
            bear.set_torque_enable((motor_id, 1))
            try:
                direct_velocity_pulse(bear, motor_id, +PULSE_VELOCITY_RAD_S, "+ pulse")
                direct_velocity_pulse(bear, motor_id, -PULSE_VELOCITY_RAD_S, "- pulse")
            finally:
                bear.set_goal_velocity((motor_id, 0.0))
                time.sleep(0.05)
                bear.set_torque_enable((motor_id, 0))
    finally:
        bear.close_port()


def mode_monitor(ids_text):
    ids = parse_ids(ids_text)
    bear = open_bear()
    try:
        live_ids = []
        for motor_id in ids:
            ok, data = ping_id(bear, motor_id)
            if ok:
                live_ids.append(motor_id)
                print(f"id {motor_id} responded: {data}")
            else:
                print(f"id {motor_id} no response: {data}")
        if not live_ids:
            print("No responding IDs to monitor.")
            return

        print("\nNo motor commands are sent.")
        print("Move pitch motor by hand, then roll motor by hand.")
        print("The changing id is the encoder/motor channel for that physical axis.")
        print("Press Ctrl+C to stop.\n")

        while True:
            print("\033[H\033[J", end="")
            print("BEAR encoder monitor")
            for motor_id in live_ids:
                try:
                    pos = bear.get_present_position(motor_id)[0][0][0]
                    vel = bear.get_present_velocity(motor_id)[0][0][0]
                    print(f"  id {motor_id:3d}: pos={pos:+9.4f} rad  vel={vel:+9.4f} rad/s")
                except Exception as exc:
                    print(f"  id {motor_id:3d}: read failed ({exc})")
            time.sleep(1.0 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print()
    finally:
        bear.close_port()


def read_positions(bear, ids):
    vals = {}
    for motor_id in ids:
        try:
            vals[motor_id] = bear.get_present_position(motor_id)[0][0][0]
        except Exception:
            vals[motor_id] = None
    return vals


def print_deltas(before, after):
    parts = []
    for motor_id in before:
        if before[motor_id] is None or after[motor_id] is None:
            parts.append(f"id{motor_id}: n/a")
        else:
            parts.append(f"id{motor_id}: {after[motor_id] - before[motor_id]:+8.4f} rad")
    print("  encoder delta:", "  ".join(parts))


def mode_coupling(enable, ids_text):
    if not enable:
        print("Coupling test is disabled by default.")
        print("Run: python3 test_control3_axes.py coupling --enable --ids 1,2")
        print("Lift/support the robot first. This sends real velocity commands.")
        return

    ids = parse_ids(ids_text)
    bear = open_bear()
    try:
        live_ids = []
        for motor_id in ids:
            ok, data = ping_id(bear, motor_id)
            if ok:
                live_ids.append(motor_id)
                print(f"id {motor_id} responded: {data}")
            else:
                print(f"id {motor_id} no response: {data}")
        if not live_ids:
            print("No responding IDs to test.")
            return

        for motor_id in live_ids:
            bear.set_mode((motor_id, 1))
            bear.set_limit_i_max((motor_id, 1.5))
            bear.set_goal_velocity((motor_id, 0.0))
            bear.set_torque_enable((motor_id, 1))

        print("\nEach pulse commands one ID, then reports encoder deltas for all IDs.")
        print("Watch physical motion too: pitch motor, roll motor, or no motion.\n")
        try:
            for command_id in live_ids:
                for velocity in (+PULSE_VELOCITY_RAD_S, -PULSE_VELOCITY_RAD_S):
                    input(f"Press Enter: command id {command_id} velocity={velocity:+.2f}")
                    before = read_positions(bear, live_ids)
                    bear.set_goal_velocity((command_id, velocity))
                    time.sleep(PULSE_TIME_S)
                    bear.set_goal_velocity((command_id, 0.0))
                    time.sleep(0.2)
                    after = read_positions(bear, live_ids)
                    print_deltas(before, after)
        finally:
            for motor_id in live_ids:
                bear.set_goal_velocity((motor_id, 0.0))
                time.sleep(0.02)
                bear.set_torque_enable((motor_id, 0))
    finally:
        bear.close_port()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("imu", "lqr", "motor", "velocity", "scan", "idpulse", "monitor", "coupling"))
    parser.add_argument("--enable", action="store_true", help="allow real motor torque pulses")
    parser.add_argument("--ids", help="comma/range list, e.g. 1,2,3 or 0-10")
    args = parser.parse_args()

    if args.mode == "imu":
        mode_imu()
    elif args.mode == "lqr":
        mode_lqr()
    elif args.mode == "motor":
        mode_motor(args.enable)
    elif args.mode == "velocity":
        mode_velocity(args.enable)
    elif args.mode == "scan":
        mode_scan(args.ids)
    elif args.mode == "idpulse":
        mode_idpulse(args.enable, args.ids)
    elif args.mode == "monitor":
        mode_monitor(args.ids)
    elif args.mode == "coupling":
        mode_coupling(args.enable, args.ids)


if __name__ == "__main__":
    main()
