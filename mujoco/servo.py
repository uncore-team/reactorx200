import numpy as np
from controller import Controller

class Servo:
    '''
    Class representing a servo motor.
    Provides methods to control and get the status of the servo.
    '''

    class LinearConverter:
        '''
        Helper class to convert between system units and application units linearly.
        '''
        def __init__(self, sys_range: list[float], app_range: list[float]):
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

    def __init__(self,
                controller: Controller,
                servo_id: int,
                pos_sys_range: list[float],
                pos_app_range: list[float],
                vel_sys_range: list[float],
                vel_app_range: list[float],
                tor_sys_range: list[float],
                tor_app_range: list[float],
                position_limits: list[float] = [-30, 30],
                velocity_limits: list[float] = [1, 15],
                home_position: float = 0,
                safe_velocity: float = 10,
                reverse_mode: bool = False):
        '''
        Initializes a servo instance. Here, safe values are used as default to avoid hardware problems.

        Parameters:
            controller (Controller): The controller used to communicate with the servo.
            servo_id (int): The ID of the servo.
            pos_sys_range (list): Position limits in system units.
            pos_app_range (list): Position limits in degrees.
            vel_sys_range (list): Velocity limits in system units.
            vel_app_range (list): Velocity limits in RPM.
            tor_sys_range (list): Torque/Force limits in system units.
            tor_app_range (list): Torque/Force limits in N/m.
            position_limits (list): Position limits (min, max) in degrees.
            velocity_limits (list): Velocity limits (min, max) in RPM ('0' means maximum velocity for Dynamixel servos).
            home_position (float): Home position in degrees.
            safe_velocity (float): Safe velocity in RPM.
            reverse_mode (bool): Whether the servo operates in reverse mode.
        '''
        self.controller = controller
        self.servo_id = servo_id

        self.position_limits = position_limits
        self.velocity_limits = velocity_limits

        self.home_position = home_position
        self.safe_velocity = safe_velocity
        self.reverse_mode = reverse_mode

        self.position = self.LinearConverter(
            sys_range=pos_sys_range,
            app_range=pos_app_range
        )
        self.velocity = self.LinearConverter(
            sys_range=vel_sys_range,
            app_range=vel_app_range
        )
        self.force = self.LinearConverter(
            sys_range=tor_sys_range,
            app_range=tor_app_range
        )

    def get_position_limits(self) -> list[float]:
        '''
        Gets the position limits of the servo.

        Returns:
            list: Position limits (min, max) in degrees.
        '''
        return self.position_limits

    def get_velocity_limits(self) -> list[float]:
        '''
        Gets the velocity limits of the servo.

        Returns:
            list: Velocity limits (min, max) in RPM.
        '''
        return self.velocity_limits

    def get_id(self) -> int:
        '''
        Gets the ID of the servo.

        Returns:
            int: The ID of the servo.
        '''
        return self.servo_id

    def get_home_position(self) -> float:
        '''
        Gets the home position of the servo.

        Returns:
            float: The current home position in degrees.
        '''
        return self.home_position

    def get_safe_velocity(self) -> float:
        '''
        Gets the maximum velocity allowed of the servo.

        Returns:
            float: The current maximum velocity in RPM.
        '''
        return self.safe_velocity

    def get_reverse_mode(self) -> bool:
        '''
        Gets the reverse mode status of the servo.

        Returns:
            bool: True if reverse mode is enabled, False otherwise.
        '''
        return self.reverse_mode

    def valid_position(self, position: float) -> bool:
        '''
        Checks if the given position is within limits.

        Parameters:
            position (float): The position to check.

        Returns:
            bool: True if the position is within limits, false otherwise.
        '''
        return (self.position_limits[0] <= position <= self.position_limits[1])

    def valid_velocity(self, velocity: float) -> bool:
        '''
        Checks if the given velocity is within limits.

        Parameters:
            velocity (float): The velocity to check.

        Returns:
            bool: True if the velocity is within limits, false otherwise.
        '''
        return (self.velocity_limits[0] <= velocity <= self.velocity_limits[1])

    def set_torque(self, value: bool):
        '''
        Enables/Disables position control for the servo.

        Parameters:
            value (bool): True to enable torque, False to disable.
        '''
        self.controller.set_torque(self.servo_id, value)

    def get_torque(self) -> bool:
        '''
        Gets the position control status for the servo.

        Returns:
            bool: True if torque is enabled, False otherwise.
        '''
        return self.controller.get_torque(self.servo_id)

    def get_force(self) -> float:
        '''
        Gets the current force of the servo in percentage.

        Returns:
            float: The current force in percentage.
        '''
        force = self.controller.get_force(self.servo_id)
        return self.force.to_app_units(force)

    def set_velocity(self, velocity: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Parameters:
            velocity (float): Velocity in RPM.
        '''
        if not self.valid_velocity(velocity):
            raise ValueError(f'RPM ({velocity}) out of range [{self.velocity_limits}] for servo {self.servo_id}')

        self.controller.set_velocity(self.servo_id, self.velocity.to_sys_units(velocity))

    def get_velocity(self) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Returns:
            float: The current velocity in RPM.
        '''
        velocity = self.controller.get_velocity(self.servo_id)
        return self.velocity.to_app_units(velocity)

    def set_position(self, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
            position (float): Target position in degrees.
        '''
        if not self.valid_position(position):
            raise ValueError(f'Position ({position}) out of range [{self.position_limits}] for servo {self.servo_id}')

        factor = -1 if self.reverse_mode else 1
        self.controller.set_position(self.servo_id, self.position.to_sys_units(factor * position))

    def get_position(self) -> float:
        '''
        Gets the current position of the servo in degrees.

        Returns:
            float: The current position of the joint in degrees.
        '''
        position = self.controller.get_position(self.servo_id)  # servo units
        factor = -1 if self.reverse_mode else 1
        return factor * self.position.to_app_units(position)
