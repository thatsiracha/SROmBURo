""""
import numpy as np
import time

from Omburo import Omburo

#Define Kp, Ki, Kd

Kp = 1.0
Ki = 0.1
Kd = 0.05

# Initialize Omburo
omburo = Omburo()

print("Starting Omburo test sequence...")
print("=" * 60)

BEAR_connected = omburo.getError()
if not BEAR_connected:
    print("Error: Unable to connect to BEAR. Please check connections and try again.")
    exit(1)

# Enable torque
print("Enabling torque...")
omburo.toggleTorque(1)

ErrorX = 
ErrorY = 
"""

"""
OmBURo PID Torque Controller
Uses: MicrostrainIMU (microstrain_imu.py) + Omburo (Omburo.py)

Architecture:
  IMU → complementary filter → tilt angles
  tilt error → PID → torque command → Omburo motors

Tuning order:
  1. Run with KP=5, KD=0.1, KI=0. Robot should resist tipping.
  2. Raise KP until it actively balances (try 10, 15, 20).
  3. If it oscillates, raise KD (0.2, 0.3, 0.5).
  4. Once stable, add KI=0.1 to correct steady lean.
  5. Adjust KP_VEL (0.2-1.0) to slow wheel runaway.
"""

import time
import signal
import sys
import math
import numpy as np
from IMURead import MicrostrainIMU, IMUReading
from Omburo import Omburo


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION — edit these values
# ══════════════════════════════════════════════════════════════════════════════

class Config:
    # Hardware ports
    IMU_PORT   = "/dev/ttyACM0"
    MOTOR_PORT = "/dev/ttyUSB0"   # set inside Omburo.py

    # Loop rate
    LOOP_HZ    = 200              # Hz
    DT         = 1.0 / LOOP_HZ   # seconds per tick

    # IMU mounting
    # True  → IMU Z-axis points DOWN  (accel_z ≈ -9.8 at rest)
    # False → IMU Z-axis points UP    (accel_z ≈ +9.8 at rest)
    IMU_INVERTED = True

    # Complementary filter — how much to trust gyro vs accelerometer
    # 0.98 = mostly gyro (fast), small accel correction (drift compensation)
    COMP_ALPHA = 0.98

    # ── PID gains (longitudinal axis) ────────────────────────────────────────
    # Output units: Nm at the wheel
    KP_L  = 15.0   # proportional on tilt angle  (Nm/rad)
    KI_L  =  0.0   # integral on tilt angle       (Nm/rad·s)  — start at 0
    KD_L  =  0.3   # derivative on tilt rate      (Nm·s/rad)

    # ── PID gains (lateral axis) ─────────────────────────────────────────────
    KP_LAT = 15.0
    KI_LAT =  0.0
    KD_LAT =  0.3

    # ── Velocity damping ─────────────────────────────────────────────────────
    # Adds a braking torque proportional to wheel speed to stop runaway rolling
    KP_VEL_L   = 0.5   # Nm/(rad/s)
    KP_VEL_LAT = 0.5

    # ── Safety limits ────────────────────────────────────────────────────────
    MAX_TORQUE_NM   = 0.5    # Nm per motor — BEAR limit_i_max=1.5 A × kt=0.35 = 0.525 Nm
    MIN_TORQUE_NM   = 0.05   # Nm — below this motors don't move; send 0 instead
    MAX_TILT_DEG    = 45.0   # degrees — cut torque if robot has fallen over
    INTEGRATOR_CAP  = 0.3    # Nm — anti-windup clamp on integral term

    # ── Tilt deadband ────────────────────────────────────────────────────────
    # Ignore tilt angles smaller than this (sensor noise suppression)
    DEADBAND_RAD = math.radians(0.5)   # 0.5 degrees

    # ── Debug printing ───────────────────────────────────────────────────────
    PRINT_EVERY = 40   # print every N ticks (~5 Hz at 200 Hz)


# ══════════════════════════════════════════════════════════════════════════════
#  TILT ESTIMATOR
# ══════════════════════════════════════════════════════════════════════════════

class TiltEstimator:
    """
    Complementary filter combining IMU gyroscope and accelerometer.

    State:
        theta_l   — longitudinal tilt (rad) — positive = top leans forward
        theta_lat — lateral tilt (rad)      — positive = top leans left

    The filter trusts the gyroscope for fast dynamics and the accelerometer
    for slow drift correction. COMP_ALPHA controls the blend (0.98 → 98% gyro).
    """

    def __init__(self, cfg: Config):
        self.cfg       = cfg
        self.theta_l   = 0.0
        self.theta_lat = 0.0

    def update(self, reading: IMUReading, dt: float) -> tuple[float, float, float, float]:
        """
        Returns (theta_l, dtheta_l, theta_lat, dtheta_lat) in radians and rad/s.
        """
        ax, ay, az = reading.accel_x, reading.accel_y, reading.accel_z
        gx, gy     = reading.gyro_x,  reading.gyro_y

        # Correct for inverted mounting
        if self.cfg.IMU_INVERTED:
            az = -az
            gy = -gy

        # Accelerometer angle estimate (only valid when nearly static)
        theta_l_acc   = math.atan2(ax, az)
        theta_lat_acc = math.atan2(ay, az)

        # Complementary filter integration
        a = self.cfg.COMP_ALPHA
        self.theta_l   = a * (self.theta_l   + gy * dt) + (1.0 - a) * theta_l_acc
        self.theta_lat = a * (self.theta_lat + gx * dt) + (1.0 - a) * theta_lat_acc

        return self.theta_l, gy, self.theta_lat, gx

    def reset(self):
        self.theta_l   = 0.0
        self.theta_lat = 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  PID CONTROLLER (per axis)
# ══════════════════════════════════════════════════════════════════════════════

class PIDAxis:
    """
    Single-axis PID controller with anti-windup and derivative filtering.
    Output is a torque command in Nm.
    """

    DERIV_FILTER_ALPHA = 0.7   # low-pass on derivative (0=none, 1=heavy)

    def __init__(self, kp: float, ki: float, kd: float,
                 integrator_cap: float, dt: float):
        self.kp  = kp
        self.ki  = ki
        self.kd  = kd
        self.cap = integrator_cap
        self.dt  = dt

        self._integral    = 0.0
        self._prev_error  = 0.0
        self._deriv_filt  = 0.0

    def compute(self, error: float, rate: float) -> float:
        """
        error : setpoint − measurement (rad)
        rate  : derivative of measurement from gyroscope (rad/s)
                (using gyro directly is more accurate than finite-difference)
        Returns torque command (Nm).
        """
        # Proportional
        p = self.kp * error

        # Integral with anti-windup clamp
        self._integral = np.clip(
            self._integral + error * self.dt,
            -self.cap, self.cap
        )
        i = self.ki * self._integral

        # Derivative from gyroscope (negative: gyro measures rate of change)
        raw_d = -self.kd * rate
        self._deriv_filt = (self.DERIV_FILTER_ALPHA * self._deriv_filt +
                            (1.0 - self.DERIV_FILTER_ALPHA) * raw_d)
        d = self._deriv_filt

        return p + i + d

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._deriv_filt = 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  MOTOR MIXER
# ══════════════════════════════════════════════════════════════════════════════

class MotorMixer:
    """
    Combines longitudinal and lateral torque commands into per-motor commands.

    OmBURo mixing convention:
        tau_wheel  = (u_long + u_lat) / 2
        tau_roller = (u_long - u_lat) / 2
    """

    def __init__(self, cfg: Config, robot: Omburo):
        self.cfg   = cfg
        self.robot = robot

    def send(self, u_long: float, u_lat: float) -> tuple[float, float]:
        tau_wheel  = (u_long + u_lat) / 2.0
        tau_roller = (u_long - u_lat) / 2.0

        # Saturate
        tau_wheel  = np.clip(tau_wheel,  -self.cfg.MAX_TORQUE_NM, self.cfg.MAX_TORQUE_NM)
        tau_roller = np.clip(tau_roller, -self.cfg.MAX_TORQUE_NM, self.cfg.MAX_TORQUE_NM)

        # Threshold: send zero if below motor noise floor
        if abs(tau_wheel)  < self.cfg.MIN_TORQUE_NM: tau_wheel  = 0.0
        if abs(tau_roller) < self.cfg.MIN_TORQUE_NM: tau_roller = 0.0

        self.robot.setTorque(tau_wheel, tau_roller)
        return tau_wheel, tau_roller

    def stop(self):
        self.robot.setTorque(0.0, 0.0)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN CONTROLLER
# ══════════════════════════════════════════════════════════════════════════════

class OmBUROController:
    """
    Top-level PID torque controller for OmBURo.

    Startup sequence:
        1. Connect IMU, wait for valid data
        2. Check motor comms
        3. Tilt sanity check (confirm angles near zero)
        4. Motor smoke test (send small pulse, check readback)
        5. Enter 200 Hz control loop
    """

    def __init__(self):
        self.cfg      = Config()
        self.imu      = MicrostrainIMU(port=self.cfg.IMU_PORT,
                                       sample_rate_hz=200,
                                       background=True)
        self.robot    = Omburo()
        self.estimator = TiltEstimator(self.cfg)
        self.pid_l    = PIDAxis(self.cfg.KP_L,  self.cfg.KI_L,  self.cfg.KD_L,
                                self.cfg.INTEGRATOR_CAP, self.cfg.DT)
        self.pid_lat  = PIDAxis(self.cfg.KP_LAT, self.cfg.KI_LAT, self.cfg.KD_LAT,
                                self.cfg.INTEGRATOR_CAP, self.cfg.DT)
        self.mixer    = MotorMixer(self.cfg, self.robot)

        self._running = False
        signal.signal(signal.SIGINT,  self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)

    # ── Startup ───────────────────────────────────────────────────────────────

    def start(self):
        print("=" * 60)
        print("  OmBURo PID Torque Controller")
        print("=" * 60)

        self._connect_imu()
        self._check_motors()
        self._tilt_sanity_check()
        self._motor_smoke_test()

        self.robot.toggleTorque(1)
        time.sleep(0.2)

        print(f"\n[Controller] Loop rate : {self.cfg.LOOP_HZ} Hz")
        print(f"[Controller] KP={self.cfg.KP_L}  KI={self.cfg.KI_L}  "
              f"KD={self.cfg.KD_L}  MAX_TAU={self.cfg.MAX_TORQUE_NM} Nm")
        print("[Controller] Running — Ctrl-C to stop.\n")

        self._running = True
        self._loop()

    # ── IMU connection ────────────────────────────────────────────────────────

    def _connect_imu(self):
        print("\n[IMU] Connecting...")
        if not self.imu.connect():
            print("[IMU] Failed — check /dev/ttyACM0"); sys.exit(1)

        print("[IMU] Waiting for first valid reading...")
        t0 = time.time()
        while not self.imu.latest.valid:
            if time.time() - t0 > 5.0:
                print("[IMU] Timeout — no data after 5 s")
                self.imu.disconnect(); sys.exit(1)
            time.sleep(0.01)
        r = self.imu.latest
        print(f"[IMU] Ready.  "
              f"accel=({r.accel_x:+.2f},{r.accel_y:+.2f},{r.accel_z:+.2f}) m/s²  "
              f"gyro=({r.gyro_x:+.3f},{r.gyro_y:+.3f},{r.gyro_z:+.3f}) rad/s")

    # ── Motor check ───────────────────────────────────────────────────────────

    def _check_motors(self):
        print("\n[Motors] Pinging BEAR actuators...")
        if not self.robot.getError():
            print("[Motors] Ping failed — check /dev/ttyUSB0 and power")
            self.imu.disconnect(); sys.exit(1)
        print("[Motors] Both motors responding.")

    # ── Tilt sanity ───────────────────────────────────────────────────────────

    def _tilt_sanity_check(self):
        print("\n[Tilt] Sampling 0.5 s of resting angles — keep robot still...")
        samples = []
        t0 = time.time()
        while time.time() - t0 < 0.5:
            r = self.imu.latest
            if r.valid:
                ax, ay, az = r.accel_x, r.accel_y, r.accel_z
                if self.cfg.IMU_INVERTED: az = -az
                samples.append((math.degrees(math.atan2(ax, az)),
                                math.degrees(math.atan2(ay, az))))
            time.sleep(0.005)

        if not samples:
            print("[Tilt] No samples — cannot verify."); return

        avg_l, avg_lat = np.mean(samples, axis=0)
        std_l, std_lat = np.std(samples,  axis=0)
        print(f"[Tilt] theta_l  = {avg_l:+.2f}° ± {std_l:.2f}°")
        print(f"[Tilt] theta_lat= {avg_lat:+.2f}° ± {std_lat:.2f}°")

        if abs(avg_l) > 20 or abs(avg_lat) > 20:
            print("[Tilt] WARNING: angles > 20° — robot may not be upright, "
                  "or set IMU_INVERTED = False in Config.")
        elif abs(avg_l) > 5 or abs(avg_lat) > 5:
            print("[Tilt] NOTE: angles > 5° — robot is leaning. "
                  "Balance it upright before releasing.")
        else:
            print("[Tilt] Angles look good.")

    # ── Motor smoke test ──────────────────────────────────────────────────────

    def _motor_smoke_test(self):
        print("\n[Smoke] Sending 0.2 Nm to both motors for 0.3 s...")
        self.robot.toggleTorque(1)
        time.sleep(0.1)

        t0 = time.time()
        while time.time() - t0 < 0.3:
            self.robot.setTorque(0.2, 0.2)
            time.sleep(0.005)
        self.robot.setTorque(0.0, 0.0)
        time.sleep(0.1)

        _, vw, _, vr = self.robot.readback()
        print(f"[Smoke] Post-pulse velocity: wheel={vw:+.4f} rad/s  roller={vr:+.4f} rad/s")

        if abs(vw) < 0.005 and abs(vr) < 0.005:
            print("[Smoke] WARNING: no velocity response — verify:")
            print("        • pybear mode=1 (current control)")
            print("        • kt=0.35 correct for your motor")
            print("        • BEAR firmware version compatible with pybear")
            print("        Continuing anyway — gains may need to be higher.")
        else:
            print("[Smoke] Motors confirmed responding.")

        self.robot.toggleTorque(0)
        time.sleep(0.1)

    # ── Main 200 Hz loop ──────────────────────────────────────────────────────

    def _loop(self):
        tick = 0
        dt   = self.cfg.DT

        while self._running:
            t0 = time.perf_counter()
            tick += 1

            # 1. Read sensors ─────────────────────────────────────────────────
            reading = self.imu.latest
            _, vel_wheel, _, vel_roller = self.robot.readback()

            # 2. Guard: invalid IMU ───────────────────────────────────────────
            if not reading.valid:
                self.mixer.stop()
                time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
                continue

            # 3. Tilt estimation ──────────────────────────────────────────────
            theta_l, dtheta_l, theta_lat, dtheta_lat = \
                self.estimator.update(reading, dt)

            # 4. Safety cutoff: fallen over ───────────────────────────────────
            max_rad = math.radians(self.cfg.MAX_TILT_DEG)
            if abs(theta_l) > max_rad or abs(theta_lat) > max_rad:
                self.mixer.stop()
                if tick % self.cfg.PRINT_EVERY == 0:
                    print(f"[Safety] Tilt exceeded {self.cfg.MAX_TILT_DEG}° — "
                          f"theta_l={math.degrees(theta_l):.1f}°  "
                          f"theta_lat={math.degrees(theta_lat):.1f}°  "
                          f"Motors OFF.")
                time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
                continue

            # 5. PID on tilt (setpoint = 0 rad = upright) ─────────────────────
            def db(v):
                # Deadband: zero out small angles so motor doesn't chatter
                return 0.0 if abs(v) < self.cfg.DEADBAND_RAD else v

            u_long = (self.pid_l.compute(db(theta_l),    dtheta_l) +
                      self.cfg.KP_VEL_L   * vel_wheel)

            u_lat  = (self.pid_lat.compute(db(theta_lat), dtheta_lat) +
                      self.cfg.KP_VEL_LAT * vel_roller)

            # 6. Mix and send ─────────────────────────────────────────────────
            tau_w, tau_r = self.mixer.send(u_long, u_lat)

            # 7. Debug print ──────────────────────────────────────────────────
            if tick % self.cfg.PRINT_EVERY == 0:
                self._print(tick * dt, theta_l, dtheta_l, theta_lat,
                            vel_wheel, u_long, u_lat, tau_w, tau_r)

            # 8. Timing ───────────────────────────────────────────────────────
            elapsed = time.perf_counter() - t0
            if elapsed > dt * 1.2:
                print(f"[Loop] Overrun tick {tick}: {elapsed*1000:.1f} ms "
                      f"(budget {dt*1000:.0f} ms)")
            time.sleep(max(0.0, dt - elapsed))

    # ── Pretty print ──────────────────────────────────────────────────────────

    def _print(self, t, tl, dtl, tlat, vw, ul, ulat, tw, tr):
        print(
            f"[t={t:6.1f}s] "
            f"tl={math.degrees(tl):+6.2f}°  "
            f"dtl={math.degrees(dtl):+6.1f}°/s  "
            f"tlat={math.degrees(tlat):+6.2f}°  "
            f"vw={vw:+.3f}  "
            f"| u=({ul:+.3f},{ulat:+.3f}) Nm  "
            f"tau_w={tw:+.3f}  tau_r={tr:+.3f} Nm"
        )

    # ── Graceful shutdown ─────────────────────────────────────────────────────

    def _shutdown(self, sig=None, frame=None):
        if not self._running:
            return
        self._running = False
        print("\n[Controller] Shutting down...")
        try:
            self.robot.setTorque(0.0, 0.0)
            time.sleep(0.05)
            self.robot.toggleTorque(0)
        except Exception:
            pass
        try:
            self.imu.disconnect()
        except Exception:
            pass
        try:
            self.robot.close()
        except Exception:
            pass
        print("[Controller] Done.")
        sys.exit(0)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    controller = OmBUROController()
    controller.start()