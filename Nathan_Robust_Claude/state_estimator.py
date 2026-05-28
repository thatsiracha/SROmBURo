import numpy as np
from microstrain_imu import IMUReading

class StateEstimator:
    """
    Complementary filter fusing IMU + wheel encoder velocities.

    IMU mounting correction
    -----------------------
    The raw output shows accel_z ≈ -9.8 m/s² at rest, which means the IMU
    is mounted with its Z-axis pointing DOWN (inverted relative to the assumed
    convention where Z points UP).  We flip accel_z and gyro_y signs so that:
      - accel_z ≈ +9.8 when upright
      - theta_l  ≈  0  when upright  (not π)

    If the robot is physically upright and theta_l is still wrong after this
    fix, set FLIP_IMU = False and instead set THETA_L_OFFSET to the observed
    resting value so the controller works from a zeroed angle.
    """

    ALPHA    = 0.98    # complementary filter: gyro trust (0–1)
    FLIP_IMU = True    # set False if accel_z is already positive at rest

    def __init__(self):
        self.theta_long   = 0.0
        self.theta_lat    = 0.0
        self.dtheta_long  = 0.0
        self.dtheta_lat   = 0.0

    def update_from_reading(self, reading: IMUReading,
                             dpsi_long: float, dpsi_lat: float,
                             dt: float) -> np.ndarray:
        return self.update(reading.gyro, reading.accel,
                           dpsi_long, dpsi_lat, dt)

    def update(self, gyro, accel, dpsi_long, dpsi_lat, dt) -> np.ndarray:
        ax, ay, az = accel
        gx, gy, gz = gyro

        # ── Mounting correction ───────────────────────────────────────────────
        if self.FLIP_IMU:
            az = -az    # flip: makes accel_z positive when upright
            gy = -gy    # flip matching gyro axis so integration stays consistent

        # ── Accelerometer tilt estimate ───────────────────────────────────────
        # arctan2(ax, az): positive when top leans forward (+x direction)
        # arctan2(ay, az): positive when top leans left   (+y direction)
        theta_long_acc = np.arctan2(ax, az)
        theta_lat_acc  = np.arctan2(ay, az)

        # ── Complementary filter ──────────────────────────────────────────────
        self.dtheta_long = gy
        self.dtheta_lat  = gx
        self.theta_long  = (self.ALPHA * (self.theta_long + gy * dt)
                            + (1.0 - self.ALPHA) * theta_long_acc)
        self.theta_lat   = (self.ALPHA * (self.theta_lat  + gx * dt)
                            + (1.0 - self.ALPHA) * theta_lat_acc)

        return np.array([
            self.theta_long,   # theta_l    (rad) — should be ~0 when upright
            self.dtheta_long,  # dtheta_l   (rad/s)
            self.theta_lat,    # theta_lat  (rad) — should be ~0 when upright
            self.dtheta_lat,   # dtheta_lat (rad/s)
            dpsi_long,         # wheel angular velocity  (rad/s)
            dpsi_lat,          # roller angular velocity (rad/s)
        ])

    def reset(self):
        self.theta_long  = 0.0
        self.theta_lat   = 0.0
        self.dtheta_long = 0.0
        self.dtheta_lat  = 0.0