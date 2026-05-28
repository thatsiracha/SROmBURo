"""
OmBURo hybrid robust controller — main loop
Scheme: PID -> inverse dynamics (B_hat * tau + n_hat) -> robustness -> plant

Startup sanity check prints theta_l and theta_lat so you can verify
the angles are near zero before the control loop runs.
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

# ── Debug flags ────────────────────────────────────────────────────────────────
DEBUG_IMU    = True
DEBUG_MOTORS = True
DEBUG_STATE  = True
PRINT_EVERY  = 40    # print every N ticks (~5 Hz at 200 Hz loop)

# ── Hardware ───────────────────────────────────────────────────────────────────
imu    = MicrostrainIMU(port="/dev/ttyACM0", sample_rate_hz=200, background=True)
motors = MotorInterface()

# ── Control modules ────────────────────────────────────────────────────────────
estimator = StateEstimator()
pid       = PIDController()
inv_dyn   = InverseDynamics()
robust    = RobustnessTerm()

tick = 0

# ── Graceful shutdown ──────────────────────────────────────────────────────────
_shutting_down = False

def shutdown(sig=None, frame=None):
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    print("\n[Main] Shutting down...")
    try:
        motors.mix_and_send(np.zeros(2))
        time.sleep(0.05)
    except Exception:
        pass
    imu.disconnect()
    motors.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ── Startup ────────────────────────────────────────────────────────────────────
print("[Main] Connecting IMU...")
if not imu.connect():
    print("[Main] IMU failed — check /dev/ttyACM0.")
    motors.shutdown()
    sys.exit(1)

print("[Main] Checking motors...")
if not motors.check_errors():
    print("[Main] Motor ping failed — check /dev/ttyUSB0.")
    imu.disconnect()
    sys.exit(1)

print("[Main] Waiting for first valid IMU reading...")
wait_start = time.time()
while not imu.latest.valid:
    if time.time() - wait_start > 5.0:
        print("[Main] IMU timeout — no valid data after 5 s.")
        imu.disconnect()
        motors.shutdown()
        sys.exit(1)
    time.sleep(0.01)

# ── Angle sanity check ─────────────────────────────────────────────────────────
# Collect ~0.5 s of readings and print the average tilt so you can confirm
# theta_l and theta_lat are near zero before trusting the controller.
print("[Main] Checking tilt angles for 0.5 s — robot should be upright and still...")
samples = []
t_check = time.time()
while time.time() - t_check < 0.5:
    r = imu.latest
    if r.valid:
        ax, ay, az = r.accel
        if estimator.FLIP_IMU:
            az = -az
        tl  = np.degrees(np.arctan2(ax,  az))
        tlat = np.degrees(np.arctan2(ay, az))
        samples.append((tl, tlat))
    time.sleep(0.005)

if samples:
    avg_tl, avg_tlat = np.mean(samples, axis=0)
    print(f"  Average tilt:  theta_l={avg_tl:+.2f}°  theta_lat={avg_tlat:+.2f}°")
    if abs(avg_tl) > 15.0 or abs(avg_tlat) > 15.0:
        print("  WARNING: tilt > 15° — robot may not be upright, or IMU axes need")
        print("           re-assignment.  Set FLIP_IMU=False in state_estimator.py")
        print("           and re-run to diagnose, or physically level the robot.")
    else:
        print("  Tilt looks good — continuing to control loop.")
else:
    print("  Could not collect samples — continuing anyway.")

time.sleep(0.2)   # settling delay after torque enable
print("[Main] Starting 200 Hz control loop.  Ctrl-C to stop.\n")

# ── Reference ──────────────────────────────────────────────────────────────────
def get_reference() -> np.ndarray:
    ref    = np.zeros(6)
    ref[4] = 0.0 / WHEEL_RADIUS
    ref[5] = 0.0 / ROLLER_EFF_RADIUS
    return ref

# ── Control loop ───────────────────────────────────────────────────────────────
while True:
    t0 = time.perf_counter()
    tick += 1

    # 1. Sense
    reading               = imu.latest
    vel_wheel, vel_roller = motors.read_wheel_velocities()

    # 2. IMU debug
    if DEBUG_IMU and (tick % PRINT_EVERY == 0):
        if reading.valid:
            print(
                f"[IMU]  "
                f"accel=({reading.accel_x:+6.3f}, {reading.accel_y:+6.3f}, "
                f"{reading.accel_z:+6.3f}) m/s²  "
                f"gyro=({reading.gyro_x:+6.3f}, {reading.gyro_y:+6.3f}, "
                f"{reading.gyro_z:+6.3f}) rad/s  "
                f"roll={reading.roll:+6.2f}° pitch={reading.pitch:+6.2f}°"
            )
        else:
            print("[IMU]  WARNING — reading not valid this tick")

    if not reading.valid:
        print(f"[Main] Tick {tick}: IMU invalid — holding zero torque")
        motors.mix_and_send(np.zeros(2))
        time.sleep(max(0.0, DT - (time.perf_counter() - t0)))
        continue

    # 3. State estimate
    x = estimator.update_from_reading(reading, vel_wheel, vel_roller, DT)

    if DEBUG_STATE and (tick % PRINT_EVERY == 0):
        print(
            f"[State] "
            f"theta_l={x[0]:+.4f} rad ({np.degrees(x[0]):+.2f}°)  "
            f"dtheta_l={x[1]:+.4f} rad/s  "
            f"theta_lat={x[2]:+.4f} rad ({np.degrees(x[2]):+.2f}°)  "
            f"psi_dot_l={x[4]:+.3f}  psi_dot_lat={x[5]:+.3f} rad/s"
        )

    # 4. Reference
    ref = get_reference()

    # 5. PID
    tau_pid = pid.compute(ref, x)

    # 6. Inverse dynamics
    u_id = inv_dyn.compute(tau_pid, x)

    # 7. Robustness
    u_rob = robust.compute(x, ref)

    # 8. Total
    u_total = u_id + u_rob

    # 9. Send to motors
    tau_wheel, tau_roller = motors.mix_and_send(u_total)

    if DEBUG_MOTORS and (tick % PRINT_EVERY == 0):
        print(
            f"[Motors] "
            f"u_id=({u_id[0]:+.3f}, {u_id[1]:+.3f})  "
            f"u_rob=({u_rob[0]:+.3f}, {u_rob[1]:+.3f})  "
            f"tau_wheel={tau_wheel:+.3f} Nm  tau_roller={tau_roller:+.3f} Nm"
        )

    # 10. Timing
    elapsed = time.perf_counter() - t0
    if elapsed > DT * 1.2:
        print(f"[Main] Loop overrun tick {tick}: {elapsed*1000:.1f} ms "
              f"(budget {DT*1000:.1f} ms)")
    time.sleep(max(0.0, DT - elapsed))