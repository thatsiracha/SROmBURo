"""
OmBURo hybrid robust controller — main loop
Scheme: PID outer loop -> inverse dynamics (B-hat * tau + n-hat) -> robustness -> plant
"""

import time
import signal
import sys
import numpy as np

from config           import DT, WHEEL_RADIUS, ROLLER_EFF_RADIUS
from microstrain_imu  import MicrostrainIMU
from state_estimator  import StateEstimator
from pid_controller   import PIDController
from inverse_dynamics import InverseDynamics
from robustness_term  import RobustnessTerm
from actuator_model   import ActuatorModel
from motor_interface  import MotorInterface
from Omburo import Omburo

# ── Debug flags — set False to silence once everything works ───────────────────
DEBUG_IMU    = True   # print IMU readings every tick
DEBUG_MOTORS = False  # print motor torque commands every tick
DEBUG_STATE  = True   # print estimated state every tick
PRINT_EVERY  = 40     # only print every N ticks (200 Hz -> ~5 Hz printout)

# ── Hardware ───────────────────────────────────────────────────────────────────
imu    = MicrostrainIMU(port="/dev/ttyACM0", sample_rate_hz=200, background=True)
motors = MotorInterface()

# ── Control modules ────────────────────────────────────────────────────────────
estimator = StateEstimator()
pid       = PIDController()
inv_dyn   = InverseDynamics()
robust    = RobustnessTerm()

tick = 0

# ── Shutdown ───────────────────────────────────────────────────────────────────
def shutdown(sig=None, frame=None):
    print("\n[Main] Shutting down...")
    motors.mix_and_send(np.zeros(2))   # zero torque before disabling
    time.sleep(0.05)
    imu.disconnect()
    motors.shutdown()
    Omburo().close()  # ensure clean disconnect in case of error
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ── Startup checks ─────────────────────────────────────────────────────────────
print("[Main] Connecting IMU...")
if not imu.connect():
    print("[Main] IMU failed — check /dev/ttyACM0 and power.")
    motors.shutdown()
    sys.exit(1)

print("[Main] Checking motors...")
if not motors.check_errors():
    print("[Main] Motor ping failed — check /dev/ttyUSB0 and power.")
    imu.disconnect()
    sys.exit(1)

# Wait for IMU background thread to collect its first valid reading
print("[Main] Waiting for first valid IMU reading...")
wait_start = time.time()
while not imu.latest.valid:
    if time.time() - wait_start > 5.0:
        print("[Main] IMU timeout — no valid data after 5 s. Check cable.")
        imu.disconnect()
        motors.shutdown()
        sys.exit(1)
    time.sleep(0.01)
print(f"[Main] IMU ready. First reading: accel={imu.latest.accel}  gyro={imu.latest.gyro}")

# Small delay after torque enable before sending commands
time.sleep(0.2)

print("[Main] All hardware OK. Starting 200 Hz control loop.\n")
print(f"       Debug: IMU={DEBUG_IMU}  Motors={DEBUG_MOTORS}  State={DEBUG_STATE}  (every {PRINT_EVERY} ticks)")
print("       Ctrl-C to stop.\n")

# ── Reference (replace with joystick / planner) ────────────────────────────────
def get_reference() -> np.ndarray:
    ref    = np.zeros(6)
    ref[4] = 0.0 / WHEEL_RADIUS        # v_long_ref  m/s -> wheel rad/s
    ref[5] = 0.0 / ROLLER_EFF_RADIUS   # v_lat_ref   m/s -> roller rad/s
    return ref

# ── 200 Hz control loop ────────────────────────────────────────────────────────
while True:
    t0 = time.perf_counter()
    tick += 1

    # 1. Sense ──────────────────────────────────────────────────────────────────
    reading               = imu.latest
    vel_wheel, vel_roller = motors.read_wheel_velocities()

    # 2. IMU debug print ────────────────────────────────────────────────────────
    if DEBUG_IMU and (tick % PRINT_EVERY == 0):
        if reading.valid:
            print(
                f"[IMU]  "
                f"accel=({reading.accel_x:+6.3f}, {reading.accel_y:+6.3f}, {reading.accel_z:+6.3f}) m/s²  "
                f"gyro=({reading.gyro_x:+6.3f}, {reading.gyro_y:+6.3f}, {reading.gyro_z:+6.3f}) rad/s  "
                f"roll={reading.roll:+6.2f}° pitch={reading.pitch:+6.2f}° yaw={reading.yaw:+6.2f}°"
            )
        else:
            print("[IMU]  WARNING — reading not valid this tick")

    # Safety: if IMU drops out mid-run, zero torque and wait
    if not reading.valid:
        print(f"[Main] Tick {tick}: IMU invalid — holding zero torque")
        motors.mix_and_send(np.zeros(2))
        time.sleep(max(0.0, DT - (time.perf_counter() - t0)))
        continue

    # 3. State estimate ─────────────────────────────────────────────────────────
    x = estimator.update_from_reading(reading, vel_wheel, vel_roller, DT)

    if DEBUG_STATE and (tick % PRINT_EVERY == 0):
        print(
            f"[State] "
            f"theta_l={x[0]:+6.4f} rad  dtheta_l={x[1]:+6.4f} rad/s  "
            f"theta_lat={x[2]:+6.4f} rad  dtheta_lat={x[3]:+6.4f} rad/s  "
            f"psi_dot_l={x[4]:+6.3f} rad/s  psi_dot_lat={x[5]:+6.3f} rad/s"
        )

    # 4. Reference ──────────────────────────────────────────────────────────────
    ref = get_reference()

    # 5. PID on velocity error ──────────────────────────────────────────────────
    tau_pid = pid.compute(ref, x)

    # 6. Inverse dynamics:  u_id = B_hat(q) * tau_pid + n_hat(q, q_dot) ────────
    u_id = inv_dyn.compute(tau_pid, x)

    # 7. Robustness injection:  u_rob = -rho * vers(s) ──────────────────────────
    u_rob = robust.compute(x, ref)

    # 8. Total command ──────────────────────────────────────────────────────────
    u_total = u_id + u_rob

    # 9. Mix -> actuator model -> Omburo.setTorque ──────────────────────────────
    tau_wheel, tau_roller = motors.mix_and_send(u_total)

    if DEBUG_MOTORS and (tick % PRINT_EVERY == 0):
        print(
            f"[Motors] "
            f"u_id=({u_id[0]:+6.3f}, {u_id[1]:+6.3f})  "
            f"u_rob=({u_rob[0]:+6.3f}, {u_rob[1]:+6.3f})  "
            f"tau_wheel={tau_wheel:+6.3f} Nm  tau_roller={tau_roller:+6.3f} Nm  "
            f"vel_wheel={vel_wheel:+6.3f} rad/s  vel_roller={vel_roller:+6.3f} rad/s"
        )

    # 10. Loop timing ───────────────────────────────────────────────────────────
    elapsed = time.perf_counter() - t0
    if elapsed > DT * 1.2:
        print(f"[Main] Loop overrun tick {tick}: {elapsed*1000:.1f} ms (budget {DT*1000:.1f} ms)")
    time.sleep(max(0.0, DT - elapsed))
