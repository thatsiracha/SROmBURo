"""
OmBURo hybrid robust controller — main loop
Scheme: PID outer loop → inverse dynamics (B̂·τ + n̂) → robustness injection → plant
"""

import time
import signal
import sys
import numpy as np

from SROmBURo.Nathan_Robust_Claude.config             import DT, WHEEL_RADIUS, ROLLER_EFF_RADIUS
from SROmBURo.Nathan_Robust_Claude.microstrain_imu    import MicrostrainIMU
from SROmBURo.Nathan_Robust_Claude.state_estimator    import StateEstimator
from SROmBURo.Nathan_Robust_Claude.pid_controller     import PIDController
from SROmBURo.Nathan_Robust_Claude.inverse_dynamics   import InverseDynamics
from SROmBURo.Nathan_Robust_Claude.robustness_term    import RobustnessTerm
from SROmBURo.Nathan_Robust_Claude.actuator_model     import ActuatorModel
from SROmBURo.Nathan_Robust_Claude.motor_interface    import MotorInterface

# ── Hardware ───────────────────────────────────────────────────────────────────
imu    = MicrostrainIMU(port="/dev/ttyACM0", sample_rate_hz=200, background=True)
motors = MotorInterface()

# ── Control modules ────────────────────────────────────────────────────────────
estimator  = StateEstimator()
pid        = PIDController()
inv_dyn    = InverseDynamics()
robust     = RobustnessTerm()

# ── Shutdown ───────────────────────────────────────────────────────────────────
def shutdown(sig=None, frame=None):
    print("\n[Main] Shutting down...")
    imu.disconnect()
    motors.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ── Startup checks ─────────────────────────────────────────────────────────────
print("[Main] Connecting IMU...")
if not imu.connect():
    motors.shutdown()
    sys.exit(1)

print("[Main] Checking motors...")
if not motors.check_errors():
    imu.disconnect()
    sys.exit(1)

print("[Main] All hardware OK. Starting 200 Hz control loop.\n")

# ── Reference (replace with joystick / planner) ────────────────────────────────
def get_reference() -> np.ndarray:
    ref    = np.zeros(6)
    ref[4] = 0.0 / WHEEL_RADIUS       # v_long_ref → wheel rad/s
    ref[5] = 0.0 / ROLLER_EFF_RADIUS  # v_lat_ref  → roller rad/s
    return ref

# ── 200 Hz control loop ────────────────────────────────────────────────────────
while True:
    t0 = time.perf_counter()

    # 1. Sense
    reading               = imu.latest
    vel_wheel, vel_roller = motors.read_wheel_velocities()

    if not reading.valid:
        motors.mix_and_send(np.zeros(2))
        time.sleep(max(0.0, DT - (time.perf_counter() - t0)))
        continue

    # 2. State estimate
    x = estimator.update_from_reading(reading, vel_wheel, vel_roller, DT)

    # 3. Reference
    ref = get_reference()

    # 4. PID on velocity error
    tau_pid = pid.compute(ref, x)

    # 5. Inverse dynamics:  u_id = B̂(q)·τ_pid + n̂(q,q̇)
    u_id = inv_dyn.compute(tau_pid, x)

    # 6. Robustness injection:  u_rob = −ρ·vers(s)
    u_rob = robust.compute(x, ref)

    # 7. Total command
    u_total = u_id + u_rob

    # 8. Mix → actuator model → Omburo.setTorque
    motors.mix_and_send(u_total)

    # 9. Timing
    elapsed = time.perf_counter() - t0
    if elapsed > DT * 1.1:
        print(f"[Main] Loop overrun: {elapsed*1000:.1f} ms")
    time.sleep(max(0.0, DT - elapsed))