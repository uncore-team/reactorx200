from abc import ABC, abstractmethod

class Controller(ABC):
    '''
    Interface class for servo controllers.
    Provides an interface for initializing, reading, writing, and managing servos.
    '''

    @abstractmethod
    def close(self):
        '''
        Closes the connection to the servo controller.
        '''
        pass

    @abstractmethod
    def factory(self, servo: int):
        '''
        Resets the servo to factory settings.

        Parameters:
        servo (int): The ID of the servo to reset.
        '''
        pass

    @abstractmethod
    def reboot(self, servo: int):
        '''
        Reboots the servo.

        Parameters:
        servo (int): The ID of the servo to reboot.
        '''
        pass

    @abstractmethod
    def set_torque(self, servo: int, value: bool):
        '''
        Sets the torque of the servo.

        Parameters:
        servo (int): The ID of the servo.
        value (bool): True to enable torque, False to disable.
        '''
        pass

    @abstractmethod
    def get_torque(self, servo: int) -> bool:
        '''
        Gets the torque status of the servo.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        bool: True if torque is enabled, False otherwise.
        '''
        pass

    @abstractmethod
    def get_force(self, servo: int) -> float:
        '''
        Gets the current force of the servo in percentage.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        float: The current force in percentage.
        '''
        pass

    @abstractmethod
    def set_velocity(self, servo: int, velocity: float):
        '''
        Sets the velocity of the servo.

        Parameters:
        servo (int): The ID of the servo.
        velocity (float): The velocity to set in RPM.
        '''
        pass

    @abstractmethod
    def get_velocity(self, servo: int) -> float:
        '''
        Gets the current velocity of the servo.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        float: The current velocity in RPM.
        '''
        pass

    @abstractmethod
    def set_position(self, servo: int, position: float):
        '''
        Sets the position of the servo.

        Parameters:
        servo (int): The ID of the servo.
        position (float): The position to set in degrees.
        '''
        pass

    @abstractmethod
    def get_position(self, servo: int) -> float:
        '''
        Gets the current position of the servo.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        float: The current position in degrees.
        '''
        pass

    @abstractmethod
    def get_status(self, servo: int) -> int:
        '''
        Gets the status of the servo.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        int: The status code of the servo.
        '''
        pass

    @abstractmethod
    def get_moving_status(self, servo: int) -> int:
        '''
        Gets the moving status of the servo.

        Parameters:
        servo (int): The ID of the servo.

        Returns:
        int: The moving status code of the servo.
        '''
        pass
