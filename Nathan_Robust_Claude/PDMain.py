"""
OmBURo direct tilt-balance controller
Step 1: Simple PD on tilt angle drives the motors directly.
        Get this working before re-introducing inverse dynamics.

Tuning guide (printed at startup):
  - Start with KP_TILT = 1.0, KD_TILT = 0.1
  - Increase KP_TILT until the robot resists tipping
  - Increase KD_TILT until oscillation is damped
  - Add KP_VEL slowly to correct steady lean
"""

import time
import signal
import sys
import numpy as np

from config          import DT, WHEEL_RADIUS, ROLLER_EFF_RADIUS
from microstrain_imu import MicrostrainIMU
from state_estimator import StateEstimator
from motor_interface import MotorInterface
from Omburo import Omburo

# ── Gains — TUNE THESE ────────────────────────────────────────────────────────
# Direct tilt PD (output in Nm, applied directly to motors)
KP_TILT_L   =  1.5    # proportional on longitudinal tilt (Nm/rad)
KD_TILT_L   =  0.08   # derivative  on longitudinal tilt rate (Nm/rad·s⁻¹)
KP_TILT_LAT =  1.5    # proportional on lateral tilt
KD_TILT_LAT =  0.08   # derivative  on lateral tilt rate

# Wheel velocity damping (resists runaway wheel spin)
KP_VEL_L    =  0.05   # Nm per rad/s of wheel velocity
KP_VEL_LAT  =  0.05

# Tilt angle deadband — ignore tiny angles (reduces chatter)
DEADBAND_RAD = 0.01   # ~0.6 degrees

# ── Debug ─────────────────────────────────────────────────────────────────────
DEBUG        = True
PRINT_EVERY  = 40     # ticks between prints (~5 Hz)

# ── Hardware ──────────────────────────────────────────────────────────────────
imu    = MicrostrainIMU(port="/dev/ttyACM0", sample_rate_hz=200, background=True)
motors = MotorInterface()
estimator = StateEstimator()

tick = 0
_shutting_down = False

# ── Shutdown ──────────────────────────────────────────────────────────────────
def shutdown(sig=None, frame=None):
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    print("\n[Main] Shutting down...")
    try:
        motors.mix_and_send(np.zeros(2))
        time.sleep(0.1)
    except Exception:
        pass
    imu.disconnect()
    motors.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ── Startup ───────────────────────────────────────────────────────────────────
print("[Main] Connecting IMU...")
if not imu.connect():
    motors.shutdown(); sys.exit(1)

print("[Main] Checking motors...")
if not motors.check_errors():
    imu.disconnect(); sys.exit(1)

print("[Main] Waiting for IMU...")
t_wait = time.time()
while not imu.latest.valid:
    if time.time() - t_wait > 5.0:
        print("[Main] IMU timeout."); imu.disconnect(); motors.shutdown(); sys.exit(1)
    time.sleep(0.01)

# Tilt sanity check
samples = []
t_check = time.time()
while time.time() - t_check < 0.5:
    r = imu.latest
    if r.valid:
        ax, ay, az = r.accel
        if estimator.FLIP_IMU: az = -az
        samples.append((np.degrees(np.arctan2(ax, az)),
                        np.degrees(np.arctan2(ay, az))))
    time.sleep(0.005)

avg_tl, avg_tlat = np.mean(samples, axis=0) if samples else (0, 0)
print(f"[Main] Resting tilt: theta_l={avg_tl:+.2f}°  theta_lat={avg_tlat:+.2f}°")
if abs(avg_tl) > 20 or abs(avg_tlat) > 20:
    print("[Main] WARNING: tilt > 20° — is the robot upright?")

Omburo().toggleTorque(1)  # pre-enable torque so first command isn't delayed by wakeup
time.sleep(0.2)

print(f"""
[Main] Starting balance loop at 200 Hz.
       Gains: KP_TILT={KP_TILT_L}  KD_TILT={KD_TILT_L}  KP_VEL={KP_VEL_L}
       Ctrl-C to stop.
""")

# ── Control law ───────────────────────────────────────────────────────────────
def apply_deadband(val, db):
    if abs(val) < db:
        return 0.0
    return val - np.sign(val) * db

# ── 200 Hz loop ───────────────────────────────────────────────────────────────


while True:
    t0 = time.perf_counter()
    tick += 1

    reading               = imu.latest
    vel_wheel, vel_roller = motors.read_wheel_velocities()

    if not reading.valid:
        motors.mix_and_send(np.zeros(2))
        time.sleep(max(0.0, DT - (time.perf_counter() - t0)))
        continue

    x = estimator.update_from_reading(reading, vel_wheel, vel_roller, DT)
    theta_l, dtheta_l, theta_lat, dtheta_lat, psi_l, psi_lat = x

    # ── Direct tilt PD ────────────────────────────────────────────────────────
    # Positive theta_l means top leans forward → drive wheel forward to catch
    tilt_l   = apply_deadband(theta_l,   DEADBAND_RAD)
    tilt_lat = apply_deadband(theta_lat, DEADBAND_RAD)

    u_long = (KP_TILT_L   * tilt_l   +
              KD_TILT_L   * dtheta_l +
              KP_VEL_L    * psi_l)

    u_lat  = (KP_TILT_LAT * tilt_lat  +
              KD_TILT_LAT * dtheta_lat +
              KP_VEL_LAT  * psi_lat)

    u_total = np.array([u_long, u_lat])

    tau_wheel, tau_roller = motors.mix_and_send(u_total)

    if DEBUG and (tick % PRINT_EVERY == 0):
        print(
            f"[t={tick*DT:6.1f}s] "
            f"theta_l={np.degrees(theta_l):+6.2f}°  "
            f"dtheta_l={np.degrees(dtheta_l):+6.2f}°/s  "
            f"theta_lat={np.degrees(theta_lat):+6.2f}°  "
            f"| u=({u_long:+.3f}, {u_lat:+.3f}) Nm  "
            f"tau=({tau_wheel:+.3f}, {tau_roller:+.3f}) Nm"
        )

    elapsed = time.perf_counter() - t0
    if elapsed > DT * 1.2:
        print(f"[Main] Overrun tick {tick}: {elapsed*1000:.1f} ms")
    time.sleep(max(0.0, DT - elapsed))