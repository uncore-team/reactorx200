from abc import ABC, abstractmethod
from enum import Enum
from controller import Controller
from servo import Servo

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

class ManipulatorArm(ABC):
    '''
    Class representing a manipulator arm with multiple joints.
    Provides methods to control and get the status of the joints.
    '''
    def __init__(self, device_name: str):
        '''
        Initializes the ManipulatorArm instance.

        Parameters:
            device_name (str): The name of the device to which the manipulator arm is connected.
        '''
        self.device_name = device_name
        self.controller = None
        self.joints = {}

        self._setup() # setup robot joints

        self._set_safe_joints_velocities() # Set safe velocities for all joints

    @abstractmethod
    def _setup(self):
        '''
        Abstract method to setup the manipulator arm.
        This method should be implemented by subclasses to initialize the joints and controller.
        '''
        pass

    def _set_safe_joints_velocities(self):
        '''
        Sets safe velocities for all joints.
        Iterates through each joint and sets the velocity to a safe value.
        '''
        for joint in self.joints:
            for servo in self.joints[joint]:
                servo.set_velocity(servo.get_safe_velocity())

    def close(self):
        '''
        Closes the controller connection.
        '''
        self.controller.close()

    def get_joints_number(self) -> int:
        '''
        Get the number of joints of the arm.

        Returns:
            int: Number of joints.
        '''
        return len(self.joints)

    def move_joint_to_home(self, joint: Joint):
        '''
        Moves a joint to its home (default) position.

        Parameters:
            joint (Joint): The joint to move to its home position.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            servo.set_position(servo.get_home_position())

    def move_joints_to_home(self):
        '''
        Moves all joints to their home (default) positions.
        '''
        for joint in self.joints:
            for servo in self.joints[joint]:
                servo.set_position(servo.get_home_position())

    def enable_joint_torque(self, joint: Joint):
        '''
        Enables position control for the servo.

        Parameters:
            joint (Joint): The joint to enable torque for.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            servo.set_torque(True)

    def enable_joints_torques(self):
        '''
        Enables position control for all servos.
        '''
        for joint in self.joints:
            for servo in self.joints[joint]:
                servo.set_torque(True)

    def disable_joint_torque(self, joint: Joint):
        '''
        Disables position control for the servo.

        Parameters:
            joint (Joint): The joint to disable torque for.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            servo.set_torque(False)

    def disable_joints_torques(self):
        '''
        Disables position control for all servos.
        '''
        for joint in self.joints:
            for servo in self.joints[joint]:
                servo.set_torque(False)

    def get_joint_force(self, joint: Joint) -> float:
        '''
        Gets the current force of the servo in percentage.

        Parameters:
            joint (Joint): Joint to get the force from.

        Returns:
            float: The current force in percentage.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        return servo.get_force()

    def get_joints_forces(self) -> list[float]:
        '''
        Gets the current forces of all servos in percentage.

        Returns:
            list: A list of current forces in percentage for all joints.
        '''
        forces = [0] * len(self.joints)
        for index, joint in enumerate(self.joints):
            servo = self.joints[joint][0] # first joint servo
            forces[index] = servo.get_force()
        return forces

    def set_joint_velocity(self, joint: Joint, velocity: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Parameters:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            velocity (float): Velocity in RPM.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            servo.set_velocity(velocity)

    def set_joints_velocities(self, velocities: list[float]):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Parameters:
            velocities (list): List of velocities in RPM for each joint.
        '''
        if len(velocities) != len(self.joints):
            raise ValueError('Number of velocities must match the number of joints.')

        for index, joint in enumerate(self.joints):
            for servo in self.joints[joint]:
                servo.set_velocity(velocities[index])

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Parameters:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        if joint not in self.joints.keys():
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        return servo.get_velocity()

    def get_joints_velocities(self) -> list[float]:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        velocities = [0] * len(self.joints)
        for index, joint in enumerate(self.joints):
            servo = self.joints[joint][0] # first joint servo
            velocities[index] = servo.get_velocity()
        return velocities

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
            joint (Joint): Joint to move.
            position (float): Target position in degrees.
        '''
        if joint not in self.joints.keys():
            raise ValueError(f'Invalid joint ID: {joint}')

        for servo in self.joints[joint]:
            servo.set_position(position)

    def set_joints_positions(self, positions: list[float]):
        '''
        Sets the target positions for all joints in degrees.

        Parameters:
            positions (list): A list of positions in degrees for each joint.
        '''
        if len(positions) != len(self.joints):
            raise ValueError('Number of positions must match the number of joints.')

        for index, joint in enumerate(self.joints):
            for servo in self.joints[joint]:
                servo.set_position(positions[index])

    def get_joint_position(self, joint: Joint) -> float:
        '''
        Gets the current position of the servo in degrees.

        Parameters:
            joint (Joint): Joint to get the position of.

        Returns:
            float: The current position of the joint in degrees.
        '''
        if joint not in self.joints.keys():
            raise ValueError(f'Invalid joint ID: {joint}')

        servo = self.joints[joint][0]   # first joint servo
        return servo.get_position()

    def get_joints_positions(self) -> list[float]:
        '''
        Gets the current positions of all joints in degrees.

        Returns:
            list: A list of current positions in degrees for all joints.
        '''
        positions = [0] * len(self.joints)
        for index, joint in enumerate(self.joints):
            servo = self.joints[joint][0]  # first joint servo
            positions[index] = servo.get_position()
        return positions

    def get_joint_position_limits(self, joint: Joint) -> list[float]:
        '''
        Gets the position limits of a specific joint.

        Parameters:
            joint (Joint): Joint to get the position limits of.

        Returns:
            list: A list of position limits for the joint.
        '''
        if joint not in self.joints.keys():
            raise ValueError(f'Invalid joint ID: {joint}')

        servo = self.joints[joint][0]   # first joint servo
        return servo.get_position_limits()

    def get_joint_velocity_limits(self, joint: Joint) -> list[float]:
        '''
        Gets the velocity limits of a specific joint.

        Parameters:
            joint (Joint): Joint to get the velocity limits of.

        Returns:
            list: A list of velocity limits for the joint.
        '''
        if joint not in self.joints.keys():
            raise ValueError(f'Invalid joint ID: {joint}')

        servo = self.joints[joint][0]   # first joint servo
        return servo.get_velocity_limits()
