import numpy as np
from Omburo import Omburo
from actuator_model import ActuatorModel

class MotorInterface:
    """
    Adapter between controller torque commands and Omburo BEAR driver.
    Mixing: wheel = (long + lat)/2,  roller = (long - lat)/2
    """

    def __init__(self):
        self.robot    = Omburo()
        self.actuator = ActuatorModel()
        self.robot.toggleTorque(1)
        print("[Motors] Initialised, torque enabled.")

    def mix_and_send(self, u: np.ndarray) -> tuple[float, float]:
        """u = [u_long, u_lat] total torque command."""
        self.robot.toggleTorque(1)
        cmd_wheel  = (u[0] + u[1]) / 2.0
        cmd_roller = (u[0] - u[1]) / 2.0
        tau = self.actuator.apply(np.array([cmd_wheel, cmd_roller]))
        self.robot.setTorque(float(tau[0]), float(tau[1]))
        return float(tau[0]), float(tau[1])

    def read_wheel_velocities(self) -> tuple[float, float]:
        _, vel_wheel, _, vel_roller = self.robot.readback()
        return vel_wheel, vel_roller

    def check_errors(self) -> bool:
        return self.robot.getError()

    def shutdown(self):
        try:
            self.robot.setTorque(0.0, 0.0)
            self.robot.toggleTorque(0)
        finally:
            self.robot.close()
        print("[Motors] Shutdown.")