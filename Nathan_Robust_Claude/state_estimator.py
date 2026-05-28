# state_estimator.py  (updated)
import numpy as np
from microstrain_imu import MicrostrainIMU, IMUReading

class StateEstimator:
    """
    Complementary filter fusing IMU data with wheel encoder velocities.
    Accepts either an IMUReading (from MicrostrainIMU) or raw tuples —
    so it works in both real-hardware and simulation modes.
    """
    ALPHA = 0.98

    def __init__(self):
        self.theta_long  = 0.0
        self.theta_lat   = 0.0
        self.dtheta_long = 0.0
        self.dtheta_lat  = 0.0

    def update_from_reading(self, reading: IMUReading,
                             dpsi_long: float, dpsi_lat: float,
                             dt: float) -> np.ndarray:
        """Primary path: pass an IMUReading directly."""
        return self.update(reading.gyro, reading.accel, dpsi_long, dpsi_lat, dt)

    def update(self, gyro, accel, dpsi_long, dpsi_lat, dt) -> np.ndarray:
        """Raw-tuple path: gyro (rad/s), accel (m/s²)."""
        theta_long_acc = np.arctan2(accel[0], accel[2])
        theta_lat_acc  = np.arctan2(accel[1], accel[2])

        self.dtheta_long = gyro[1]
        self.dtheta_lat  = gyro[0]
        self.theta_long  = (self.ALPHA * (self.theta_long + gyro[1] * dt)
                            + (1 - self.ALPHA) * theta_long_acc)
        self.theta_lat   = (self.ALPHA * (self.theta_lat  + gyro[0] * dt)
                            + (1 - self.ALPHA) * theta_lat_acc)

        return np.array([
            self.theta_long, self.dtheta_long,
            self.theta_lat,  self.dtheta_lat,
            dpsi_long,       dpsi_lat,
        ])