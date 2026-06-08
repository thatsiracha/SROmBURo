"""
BEAR motor driver for SrOmBURo direct-force control (mode 3).

Parallel to OmburoVel.py — does not modify or inherit from it.
Use this module from torque/force outer loops (e.g. mpc_control_s.py).

Mode 3 (force) uses the BEAR onboard force/torque estimate and the
firmware force PID (p/i/d_gain_force) to track a goal force setpoint.
The iq current loop still runs underneath for current limiting.

setForce(force_wheel, force_roller):
    force_wheel  → motor id2 (wheel / pitch axis)  [Nm]
    force_roller → motor id1 (roller / roll axis)  [Nm]

In force mode, goal_iq carries the desired output torque [Nm], not amps.
"""

from pybear import Manager

# MOTOR PARAMETERS (match OmburoVel.py wiring)
id_wheel = 2
id_roller = 1

kt = 0.35          # Nm/A — used for readback estimate and current-limit conversion
iq = 6.0           # hardware current limit [A]

# Inner force-loop PID (mode 3). Tune on hardware if tracking feels soft/sluggish.
P_GAIN_FORCE = 1.0
I_GAIN_FORCE = 0.15
D_GAIN_FORCE = 0.0

# Underlying iq loop — keeps phase current bounded (same starting point as OmburoVel)
P_GAIN_IQ = 0.277
I_GAIN_IQ = 0.061
D_GAIN_IQ = 0.0


class OmburoForce:

    def __init__(self):
        self.bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
        self.id_wheel = id_wheel
        self.id_roller = id_roller

        self._configure_current_loop()
        self._configure_force_loop()
        self.bear.set_mode((id_wheel, 3), (id_roller, 3))
        self.bear.set_limit_i_max((id_wheel, iq), (id_roller, iq))

    def _configure_current_loop(self):
        self.bear.set_p_gain_iq((id_wheel, P_GAIN_IQ), (id_roller, P_GAIN_IQ))
        self.bear.set_i_gain_iq((id_wheel, I_GAIN_IQ), (id_roller, I_GAIN_IQ))
        self.bear.set_d_gain_iq((id_wheel, D_GAIN_IQ), (id_roller, D_GAIN_IQ))

    def _configure_force_loop(self):
        self.bear.set_p_gain_force((id_wheel, P_GAIN_FORCE), (id_roller, P_GAIN_FORCE))
        self.bear.set_i_gain_force((id_wheel, I_GAIN_FORCE), (id_roller, I_GAIN_FORCE))
        self.bear.set_d_gain_force((id_wheel, D_GAIN_FORCE), (id_roller, D_GAIN_FORCE))

    def getError(self):
        return (self.bear.ping(id_wheel)[0][1] and self.bear.ping(id_roller)[0][1])

    def toggleTorque(self, val):
        """val: 1 = ON / 0 = OFF"""
        self.bear.set_torque_enable((id_wheel, val), (id_roller, val))

    def setCurrentLimit(self, iq_max):
        self.bear.set_limit_i_max((id_wheel, iq_max), (id_roller, iq_max))

    def setForceMode(self):
        self.bear.set_mode((id_wheel, 3), (id_roller, 3))

    def setVelocityMode(self):
        self.bear.set_mode((id_wheel, 1), (id_roller, 1))

    def setTorqueMode(self):
        """Alias: torque-mode in legacy code maps to BEAR mode 0, not force mode."""
        self.bear.set_mode((id_wheel, 0), (id_roller, 0))

    def setForce(self, force_wheel: float, force_roller: float):
        """Command output torque [Nm] on each axis (force mode goal register)."""
        self.bear.set_goal_iq((id_wheel, force_wheel), (id_roller, force_roller))

    def setTorque(self, torque_wheel: float, torque_roller: float):
        """Alias for setForce — outer controllers may use either name."""
        self.setForce(torque_wheel, torque_roller)

    def readback(self):
        position_wheel = self.bear.get_present_position(id_wheel)[0][0][0]
        velocity_wheel = -self.bear.get_present_velocity(id_wheel)[0][0][0]
        position_roller = self.bear.get_present_position(id_roller)[0][0][0]
        velocity_roller = self.bear.get_present_velocity(id_roller)[0][0][0]
        return (position_wheel, velocity_wheel, position_roller, velocity_roller)

    def readPresentTorque(self):
        """Estimated output torque [Nm] from measured phase current."""
        iq_w = self.bear.get_present_iq(id_wheel)[0][0]
        iq_r = self.bear.get_present_iq(id_roller)[0][0]
        return (iq_w * kt, iq_r * kt)

    def close(self):
        self.bear.close_port()
