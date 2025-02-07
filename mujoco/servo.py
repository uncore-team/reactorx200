import numpy as np

class Servo:
    '''
    Class representing a servo motor.
    Provides methods to control and get the status of the servo.
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

    def __init__(self, servo_id:int,
                pos_sys_range:list[float, float],
                pos_app_range:list[float, float],
                vel_sys_range:list[float, float],
                vel_app_range:list[float, float],
                tor_sys_range:list[float, float],
                tor_app_range:list[float, float],
                pos_limits:list[float, float]=[-30, 30],
                vel_limits:list[float, float]=[5, 50],
                home_position:float=0,
                max_velocity:float=10,
                reverse_mode:bool=False):
        '''
        Initializes a servo instance. Here, safe values are used as default to avoid hardware problems.

        Parameters:
            servo_id (int): The ID of the servo.
            pos_sys_range (list): Position limits in system units.
            pos_app_range (list): Position limits in application units.
            vel_sys_range (list): Velocity limits in system units.
            vel_app_range (list): Velocity limits in application units.
            tor_sys_range (list): Torque/Force limits in system units.
            tor_app_range (list): Torque/Force limits in application units.            
            pos_limits (list): Position limits (min, max) in degrees.
            vel_limits (list): Velocity limits (min, max) in RPM (a value of 0 rpm means maximum velocity for Dynamixel servos).
            home_position (int): Home position in degrees.
            max_velocity (int): Maximum velocity in RPM.
            reverse (bool): Whether the servo operates in reverse mode.
        '''
        self.servo_id = servo_id
        self.pos_sys_range = pos_sys_range
        self.pos_app_range = pos_app_range
        self.vel_sys_range = vel_sys_range
        self.vel_app_range = vel_app_range
        self.tor_sys_range = tor_sys_range
        self.tor_app_range = tor_app_range
        self.pos_limits = pos_limits
        self.vel_limits = vel_limits
        self.home_position = home_position
        self.velocity = max_velocity
        self.reverse_mode = reverse_mode

        self.pos_converter = self.LinearConverter(
            sys_range=pos_sys_range,
            app_range=pos_app_range
        )
        self.vel_converter = self.LinearConverter(
            sys_range=vel_sys_range,
            app_range=vel_app_range
        )
        self.tor_converter = self.LinearConverter(
            sys_range=tor_sys_range,
            app_range=tor_app_range
        )

    def set_position_limits(self, limits:list[float]):
        '''
        Changes the position limits of the servo.

        Parameters:
            pos_limits (list): New position limits (min, max) in degrees.
        '''
        if len(limits) != 2:
            raise ValueError('Wrong range length, please check given values.')
        elif limits[0] < self.pos_app_range[0] or limits[1] > self.pos_app_range[1]:
            raise ValueError('Given limits are out of factory range, please check given values.')
        self.pos_limits = limits

    def get_position_limits(self) -> list:
        '''
        Gets the position limits of the servo.

        Returns:
            list: Position limits (min, max) in degrees.
        '''
        return self.pos_limits

    def set_velocity_limits(self, limits:list):
        '''
        Changes the velocity limits of the servo.

        Parameters:
            limits (list): New velocity limits (min, max) in RPM.
        '''
        if len(limits) != 2:
            raise ValueError('Wrong range length, please check given values.')
        elif limits[0] < self.vel_app_range[0] or limits[1] > self.vel_app_range[1]:
            raise ValueError('Given limits are out of factory range, please check given values.')
        self.vel_limits = limits

    def get_velocity_limits(self) -> list:
        '''
        Gets the velocity limits of the servo.

        Returns:
            list: Velocity limits (min, max) in RPM.
        '''
        return self.vel_limits

    def set_home_position(self, position: float):
        '''
        Sets the home position of the servo.

        Parameters:
            position (float): Target position in degrees.
        '''
        if not self.valid_position(position):
            raise ValueError('Position out of limits.')
        self.home_position = position

    def get_home_position(self) -> float:
        '''
        Gets the home position of the servo.

        Returns:
            float: The current home position in degrees.
        '''
        return self.home_position

    def set_max_velocity(self, velocity: float):
        '''
        Sets the maximum velocity of the servo.

        Parameters:
            velocity (float): Velocity in RPM.
        '''
        if not self.valid_velocity(velocity):
            raise ValueError('Velocity out of limits.')
        self.velocity = velocity

    def get_max_velocity(self) -> float:
        '''
        Gets the maximum velocity allowed of the servo.

        Returns:
            float: The current maximum velocity in RPM.
        '''
        return self.velocity

    def set_reverse_mode(self, value: bool):
        '''
        Sets the reverse mode of the servo.

        Parameters:
            value (bool): True to enable reverse mode, False to disable.
        '''
        self.reverse_mode = value

    def get_reverse_mode(self) -> bool:
        '''
        Gets the reverse mode status of the servo.

        Returns:
            bool: True if reverse mode is enabled, False otherwise.
        '''
        return self.reverse_mode

    def get_id(self) -> int:
        '''
        Gets the ID of the servo.

        Returns:
            int: The ID of the servo.
        '''
        return self.servo_id

    def pos_to_sys(self, app_units: float) -> float:
        '''
        Converts position from application units to system units.

        Parameters:
            app_units (float): Position in application units.

        Returns:
            float: Position in system units.
        '''
        return self.pos_converter.to_sys_units(app_units)

    def pos_to_app(self, sys_units: float) -> float:
        '''
        Converts position from system units to application units.

        Parameters:
            sys_units (float): Position in system units.

        Returns:
            float: Position in application units.
        '''
        return self.pos_converter.to_app_units(sys_units)

    def vel_to_sys(self, app_units: float) -> float:
        '''
        Converts velocity from application units to system units.

        Parameters:
            app_units (float): Velocity in application units.

        Returns:
            float: Velocity in system units.
        '''
        return self.vel_converter.to_sys_units(app_units)

    def vel_to_app(self, sys_units: float) -> float:
        '''
        Converts velocity from system units to application units.

        Parameters:
            sys_units (float): Velocity in system units.

        Returns:
            float: Velocity in application units.
        '''
        return self.vel_converter.to_app_units(sys_units)

    def tor_to_sys(self, app_units: float) -> float:
        '''
        Converts torque from application units to system units.

        Parameters:
            app_units (float): Torque in application units.

        Returns:
            float: Torque in system units.
        '''
        return self.tor_converter.to_sys_units(app_units)

    def tor_to_app(self, sys_units: float) -> float:
        '''
        Converts torque from system units to application units.

        Parameters:
            sys_units (float): Torque in system units.

        Returns:
            float: Torque in application units.
        '''
        return self.tor_converter.to_app_units(sys_units)

    def valid_position(self, position: float) -> bool:
        '''
        Checks if the given position is within limits.

        Parameters:
            position (float): The position to check.

        Returns:
            bool: True if the position is within limits, false otherwise.
        '''
        return (self.pos_limits[0] <= position <= self.pos_limits[1])

    def valid_velocity(self, velocity: float) -> bool:
        '''
        Checks if the given velocity is within limits.

        Parameters:
            velocity (float): The velocity to check.

        Returns:
            bool: True if the velocity is within limits, false otherwise.
        '''
        return (self.vel_limits[0] <= velocity <= self.vel_limits[1])
