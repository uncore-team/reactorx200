from enum import Enum
import numpy as np

class Joint(Enum):
    '''
    Enum class representing different joints of a robotic arm.
    '''
    Waist = 0
    Shoulder = 1
    Elbow = 2
    WristAngle = 3
    WristRotation = 4
    Gripper = 5

    @classmethod
    def is_valid(cls, joint: 'Joint') -> bool:
        '''
        Checks if the provided joint is a valid member of the Joint enum.

        Parameters:
        joint (Joint): The joint to check.

        Returns:
        bool: True if the joint is valid, False otherwise.
        '''
        return joint in cls


class UnitsConverter:
    '''
    Stores conversion values and methods for translating between application-specific units and 
    physical units for the servo. We assume here linear conversions between ranges.
    '''
    
    class LinearConverter:
        def __init__(self, sys_range, app_range):
            '''
            Initializes the LinearConverter with system and application ranges.

            Parameters:
            sys_range (list): Range of values in system units.
            app_range (list): Range of values in application units.
            '''
            self.sys_range = sys_range
            self.app_range = app_range

        def to_sys_units(self, app_units: float) -> float:
            '''
            Converts application units to system units linearly.

            Parameters:
            app_units (float): Value in application units.

            Returns:
            float: Corresponding value in system units.
            '''
            return np.interp(app_units, self.app_range, self.sys_range)

        def to_app_units(self, sys_units: int) -> float:
            '''
            Converts system units to application units linearly.

            Parameters:
            sys_units (int): Value in system units.

            Returns:
            float: Corresponding value in application units.
            '''
            return np.interp(sys_units, self.sys_range, self.app_range)

    def __init__(self, pos_sys_range, pos_app_range, vel_sys_range, vel_app_range, tor_sys_range, tor_app_range):
        '''
        Initializes factory-defined limits for MuJoCo servos.

        Parameters:
        pos_sys_range (list): Position limits in system units.
        pos_app_range (list): Position limits in application units.
        vel_sys_range (list): Velocity limits in system units.
        vel_app_range (list): Velocity limits in application units.
        tor_sys_range (list): Torque/Force limits in system units.
        tor_app_range (list): Torque/Force limits in application units.
        '''
        self.position = self.LinearConverter(
            sys_range=pos_sys_range,
            app_range=pos_app_range
        )
        self.velocity = self.LinearConverter(
            sys_range=vel_sys_range,
            app_range=vel_app_range
        )
        self.torque = self.LinearConverter(
            sys_range=tor_sys_range,
            app_range=tor_app_range
        )