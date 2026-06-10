import time
from pybear import Manager

# MOTOR PARAMETERS
id_wheel = 2
id_roller = 1

kt = 0.35
iq = 1.5

class Omburo:

    def __init__(self):
        
        self.bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)

        # Set both motors to torque/current control mode.
        # PyBEAR examples use mode 1=velocity, 2=position, 0=torque/current, 3- Direct Force Mode.
        self.bear.set_mode((id_wheel, 1),(id_roller, 1))

        # Configure motor PID
        self.bear.set_p_gain_iq((id_wheel, 0.277),(id_roller, 0.277))
        self.bear.set_i_gain_iq((id_wheel, 0.061),(id_roller, 0.061))
        self.bear.set_d_gain_iq((id_wheel, 0.0),(id_roller, 0.0))
        self.bear.set_p_gain_iq((id_wheel, 0.277),(id_roller, 0.277))
        self.bear.set_i_gain_iq((id_wheel, 0.061),(id_roller, 0.061))
        self.bear.set_d_gain_iq((id_wheel, 0.0),(id_roller, 0.0))

        self.bear.set_limit_i_max((id_wheel, 6),(id_roller, 6))

    def getError(self):
        return (self.bear.ping(id_wheel)[0][1] and self.bear.ping(id_roller)[0][1])

    def toggleTorque(self, val):
        '''
        val: 1 = ON / 0 = OFF
        '''
        self.bear.set_torque_enable((id_wheel, val),(id_roller, val))

    def setTorque(self, torque_wheel, torque_roller):
        iq_wheel = torque_wheel / kt
        iq_roller = torque_roller / kt
        self.bear.set_goal_iq((id_wheel, iq_wheel),(id_roller, iq_roller))

    def setCurrentLimit(self, iq_max):
        self.bear.set_limit_i_max((id_wheel, iq_max),(id_roller, iq_max))

    def setVelocityMode(self):
        self.bear.set_mode((id_wheel, 1),(id_roller, 1))

    def setTorqueMode(self):
        self.bear.set_mode((id_wheel, 0),(id_roller, 0))

    def setVelocity(self, vel_wheel, vel_roller):
        self.bear.set_goal_velocity((id_wheel, vel_wheel),(id_roller, vel_roller))

    def readback(self):
        # Read back the current position and velocity of the motors
        position_wheel = self.bear.get_present_position(id_wheel)[0][0][0]
        velocity_wheel = -self.bear.get_present_velocity(id_wheel)[0][0][0]
        position_roller = self.bear.get_present_position(id_roller)[0][0][0]
        velocity_roller = self.bear.get_present_velocity(id_roller)[0][0][0]

        return (position_wheel, velocity_wheel, position_roller, velocity_roller)

    def close(self):
        self.bear.close_port()
