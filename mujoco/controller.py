from abc import ABC, abstractmethod

class Controller(ABC):
    '''
    Interface class for servo controllers.
    Provides an interface for initializing, reading, writing, and managing servos.
    '''
    def __init__(self):
        '''
        Initializes a servo controller instance.

        Parameters:
        device_name (str): Device name (e.g., COM port or USB port)
        baud_rate (int): Baudrate for communication (default is 1000000)
        protocol (int): Protocol version (default is 2)
        '''

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def factory(self, servo: int):
        pass

    @abstractmethod
    def reboot(self, servo: int):
        pass

    @abstractmethod
    def set_torque(self, servo: int, value: bool):
        pass

    @abstractmethod
    def get_torque(self, servo: int) -> bool:
        pass

    @abstractmethod
    def get_force(self, servo: int) -> float:
        pass

    @abstractmethod
    def set_velocity(self, servo: int, velocity: float):
        pass

    @abstractmethod
    def get_velocity(self, servo: int) -> float:
        pass

    @abstractmethod
    def set_position(self, servo: int, position: float):
        pass

    @abstractmethod
    def get_position(self, servo: int) -> float:
        pass

    @abstractmethod
    def get_status(self, servo: int) -> int:
        pass

    @abstractmethod
    def get_moving_status(self, servo: int) -> int:
        pass
