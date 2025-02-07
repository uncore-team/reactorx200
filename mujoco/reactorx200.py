from enum import Enum

from manipulatorarm import Joint
from mujocoreactorx200 import MuJoCoReactorX200
from trossenreactorx200 import TrossenReactorX200

class ExecutionType(Enum):
    Physical=0
    Simulated=1
    DigitalTwin=2

class ReactorX200:
    def __init__(self, exec_type:ExecutionType=ExecutionType.Simulated, device_name:str=None):
        '''
        Initialize the ReactorX200 class.

        :param com_port: (str) COM port for the real robot. If None, the Mujoco model is used.
        '''
        self.joints_number = 6  # Waist, Shoulder, Elbow, WristAngle, WristRotation, Gripper
        self.exec_type = exec_type
        self.device_name = device_name

        if self.exec_type not in ExecutionType:
            raise ValueError('Invalid port name.')

        if self.exec_type in [ExecutionType.Physical, ExecutionType.DigitalTwin] and device_name is None:
            raise ValueError('Invalid port name.')

        self.robots = [] # list of arms to manage

        self._start()

    def _start(self):

        if self.exec_type in [ExecutionType.Physical, ExecutionType.DigitalTwin]:
            self.robots.append(TrossenReactorX200(self.device_name))
        if self.exec_type in [ExecutionType.Simulated, ExecutionType.DigitalTwin]:
            self.robots.append(MuJoCoReactorX200())

    def close(self):
        for robot in self.robots:
            robot.close()

    def move_joint_to_home(self, joint: Joint):
        '''
        Moves a joint to its home (default) position.
        '''
        if not joint in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.move_joint_to_home(joint)

    def move_joints_to_home(self):
        '''
        Moves all joints to their home (default) positions.
        '''
        for robot in self.robots:
            robot.move_joints_to_home()

    def enable_joint_torque(self, joint: Joint):
        '''
        Enables position control for the joint given.

        :param joint (Joint): List of torques to apply to each joint.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.enable_joint_torque(joint)

    def enable_joints_torques(self):
        '''
        Enables position control for all joints.
        '''
        for robot in self.robots:
            robot.enable_joints_torques()

    def disable_joint_torque(self, joint: Joint):
        '''
        Disables position control for the servo.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.disable_joint_torque(joint)

    def disable_joints_torques(self):
        '''
        Disables position control for all joints.
        '''
        for robot in self.robots:
            robot.disable_joints_torques()

    def set_joint_torque(self, joint: Joint, value: bool):
        '''
        Enables position control for the servo.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.set_joint_torque(joint, value)

    def set_joints_torques(self, values: list[bool]):
        '''
        Enables position control for the servo.
        '''
        if len(values) != self.joints_number:
            raise ValueError('Number of velocities must match the number of joints.')
        for robot in self.robots:
            robot.set_joints_torques(values)

    def get_joint_torque(self, joint: Joint) -> bool:
        '''
        Enables/disables position control for the servo.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        return self.robots[0].get_joint_torque(joint) # first robot

    def get_joints_torques(self) -> list[bool]:
        '''
        Gets the current torques/forces percent of all servos.

        Returns:
            list: A list of current torques/forces for all joints.
        '''
        return self.robots[0].get_joints_torques() # first robot

    def get_joint_force(self, joint: Joint) -> float:
        '''
        Gets the current torque/force percent of the joint.

        Args:
        :joint (Joint): Joint to get the torque/force from.

        Returns:
        :float: The current torque percent.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        return self.robots[0].get_joint_force(joint) # first robot

    def get_joints_forces(self) -> list[float]:
        '''
        Gets the current torques/forces percent of all servos.

        Returns:
            list: A list of current torques/forces for all joints.
        '''
        return self.robots[0].get_joints_forces()  # first robot

    def set_joint_velocity(self, joint: Joint, rpm: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Args:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            rpm (float): Velocity in RPM (0-100).
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.set_joint_velocity(joint, rpm)

    def set_joints_velocities(self, velocities:list[float]):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Args:
            velocities (list): List of velocities in RPM for each joint.
        '''
        if len(velocities) != self.joints_number:
            raise ValueError('Number of velocities must match the number of joints.')
        for robot in self.robots:
            robot.set_joints_velocities(velocities)

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Args:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        return self.robots[0].get_joint_velocity(joint)  # first robot

    def get_joints_velocities(self) -> list[float]:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        return self.robots[0].get_joints_velocities()  # first robot

    def open_gripper(self):
        '''
        Opens the gripper by moving it to its maximum position.
        '''
        for robot in self.robots:
            robot.open_gripper()

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its minimum position.
        '''
        for robot in self.robots:
            robot.close_gripper()

    def set_joint_position(self, joint:Joint, position:float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
        servo (int): Joint to move.
        position (float): The desired position in degrees.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        for robot in self.robots:
            robot.set_joint_position(joint, position)

    def set_joints_positions(self, positions:list[float]):
        '''
        Sets positions for all joints (in degrees).

        Parameters:
        positions (list): A list of positions in degrees for each joint.
        '''
        if len(positions) != self.joints_number:
            raise ValueError('Number of positions must match the number of joints.')

        for robot in self.robots:
            robot.set_joints_positions(positions)

    def get_joint_position(self, joint:Joint) -> float:
        '''
        Gets the current position of a specific joint (in degrees).

        Parameters:
        joint (int): Joint to get the position from.

        Returns:
        float: The current position of the joint in degrees.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')
        return self.robots[0].get_joint_position(joint)  # first robot

    def get_joints_positions(self) -> list[float]:
        '''
        Gets the current positions of all joints (in degrees).

        Returns:
        list: A list of current positions in degrees for all joints.
        '''
        return self.robot[0].get_joints_positions()  # first robot
