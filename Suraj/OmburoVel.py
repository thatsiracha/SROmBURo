"""
BEAR motor driver for SrOmBURo velocity-mode balancing.

Based on Omburo.py — dedicated to velocity control (mode 1).
Use this module from velocity-loop controllers (e.g. control2_Suraj.py).

setVelocity(vel_wheel, vel_roller):
    vel_wheel  → motor id2 (wheel)
    vel_roller → motor id1 (roller motor command)
"""

from pybear import Manager

# MOTOR PARAMETERS
id_wheel = 2
id_roller = 1

kt = 0.35
iq = 6.0


class OmburoVel:

    def __init__(self):

        self.bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
        self.id_wheel = id_wheel
        self.id_roller = id_roller

        # Velocity mode (1) for both motors.
        self.bear.set_mode((id_wheel, 1), (id_roller, 1))

        # Motor inner-loop PID (iq current loop)
        self.bear.set_p_gain_iq((id_wheel, 0.277), (id_roller, 0.277))
        self.bear.set_i_gain_iq((id_wheel, 0.061), (id_roller, 0.061))
        self.bear.set_d_gain_iq((id_wheel, 0.0), (id_roller, 0.0))

        self.bear.set_limit_i_max((id_wheel, 6.0), (id_roller, 6.0))

    def getError(self):
        return (self.bear.ping(id_wheel)[0][1] and self.bear.ping(id_roller)[0][1])

    def toggleTorque(self, val):
        '''
        val: 1 = ON / 0 = OFF
        '''
        self.bear.set_torque_enable((id_wheel, val), (id_roller, val))

    def setCurrentLimit(self, iq_max):
        self.bear.set_limit_i_max((id_wheel, iq_max), (id_roller, iq_max))

    def setVelocityMode(self):
        self.bear.set_mode((id_wheel, 1), (id_roller, 1))

    def setTorqueMode(self):
        self.bear.set_mode((id_wheel, 0), (id_roller, 0))

    def setVelocity(self, vel_wheel, vel_roller):
        self.bear.set_goal_velocity((id_wheel, vel_wheel), (id_roller, vel_roller))

    def readback(self):
        # Read back the current position and velocity of the motors.
        # Wheel velocity is negated to match the physical spin convention.
        position_wheel = self.bear.get_present_position(id_wheel)[0][0][0]
        velocity_wheel = -self.bear.get_present_velocity(id_wheel)[0][0][0]
        position_roller = self.bear.get_present_position(id_roller)[0][0][0]
        velocity_roller = self.bear.get_present_velocity(id_roller)[0][0][0]

        return (position_wheel, velocity_wheel, position_roller, velocity_roller)

    def close(self):
        self.bear.close_port()
