from enum import Enum
import mujoco
import mujoco.viewer

from utils import Joint
from dxlcontroller import DXLController
from mjccontroller import MJCController
from mujocoreactorx200 import TrossenReactorX200
from trossenreactorx200 import MuJoCoReactorX200

class ExecutionType(Enum):
    Physical=0
    Simulated=1
    DigitalTwin=2

class ReactorX200:
    def __init__(self, exec_type: ExecutionType = ExecutionType.Simulated, device_name:str = None):
        '''
        Initialize the ReactorX200 class.

        :param com_port: (str) COM port for the real robot. If None, the Mujoco model is used.
        '''
        self.joints_number = 6
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
            self.robots.append( RealReactorX200(DXLController(self.device_name)) )
        if self.exec_type in [ExecutionType.Simulated, ExecutionType.DigitalTwin]:
            self.robots.append( SimReactorX200(MJCController(self.robot_name)) )

        self.joints_number = self.robots[0].get_joints_number()
        if len(self.robots) == 2:
            if self.joints_number != self.robots[1].get_joints_number():
                raise Exception(f'The number of joints of the simulated and physical arms are different!.')

    def close(self):
        for robot in self.robots:
            robot.close()

    def home_joint(self, joint: Joint):
        '''
        Moves a joint to its home (default) position.
        '''
        if Joint.is_valid(joint):
            for robot in self.robots:
                robot.home_joint(joint)
        else:
            raise ValueError('Invalid joint ID')

    def home_all_joints(self):
        '''
        Moves all joints to their home (default) positions.
        '''
        for robot in self.robots:
            robot.home_all_joints()

    def enable_joint_torque(self, joint: Joint):
        '''
        Enables position control for the joint given.

        :param joint (Joint): List of torques to apply to each joint.
        '''
        if Joint.is_valid(joint):
            for robot in self.robots:
                robot.enable_joint_torque(joint)
        else:
            raise ValueError('Invalid joint ID')

    def enable_all_joints_torques(self):
        '''
        Enables position control for all joints.
        '''
        for robot in self.robots:
            robot.enable_all_joints_torques()

    def disable_joint_torque(self, joint: Joint):
        '''
        Disables position control for the servo.
        '''
        if Joint.is_valid(joint):
            for robot in self.robots:
                robot.disable_joint_torque(joint)
        else:
            raise ValueError('Invalid joint ID')

    def disable_all_joints_torques(self):
        '''
        Disables position control for all joints.
        '''
        for robot in self.robots:
            robot.disable_all_joints_torques()

    def get_joint_torque(self, joint: Joint) -> float:
        '''
        Gets the current torque/force percent of the joint.

        Args:
        :joint (Joint): Joint to get the torque/force from.

        Returns:
        :float: The current torque percent.
        '''
        if Joint.is_valid(joint):
            return self.robots[0].get_joint_torque(joint) # first robot (physical, typically)
        else:
            raise ValueError('Invalid joint ID')

    def get_all_joints_torques(self) -> list:
        '''
        Gets the current torques/forces percent of all servos.

        Returns:
            list: A list of current torques/forces for all joints.
        '''
        return self.robots[0].get_all_joints_torques()  # first robot

    def set_joint_velocity(self, joint: Joint, rpm: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Args:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            rpm (float): Velocity in RPM (0-100).
        '''
        if Joint.is_valid(joint):
            for robot in self.robots:
                robot.set_joint_velocity(joint, rpm)
        else:
            raise ValueError('Invalid joint ID')

    def set_all_joints_velocities(self, velocities: list):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Args:
            velocities (list): List of velocities in RPM for each joint.
        '''
        if len(velocities) != self.joints_number:
            raise ValueError('Number of velocities must match the number of joints.')

        for robot in self.robots:
            robot.set_all_joints_velocities(velocities)

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Args:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        if Joint.is_valid(joint):
            return self.robots[0].get_joint_velocity(joint)
        else:
            raise ValueError('Invalid joint ID')

    def get_all_joint_velocities(self) -> list:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        return self.robots[0].get_all_joints_velocities()

    def open_gripper(self):
        '''
        Opens the gripper by moving it to its home position.
        '''
        for robot in self.robots:
            robot.open_gripper()

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its home position.
        '''
        for robot in self.robots:
            robot.close_gripper()

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
        servo (int): Joint to move.
        position (float): The desired position in degrees.
        '''
        if Joint.is_valid(joint):
            for robot in self.robots:
                robot.set_joint_position(joint, position)
        else:
            raise ValueError('Invalid joint ID')

    def set_all_joint_positions(self, positions):
        '''
        Sets positions for all joints (in degrees).

        Parameters:
        positions (list): A list of positions in degrees for each joint.
        '''
        if len(positions) != self.joints_number:
            raise ValueError('Number of positions must match the number of joints.')

        for robot in self.robots:
            robot.set_all_joints_positions(positions)

    def get_joint_position(self, joint: Joint) -> float:
        '''
        Gets the current position of a specific joint (in degrees).

        Parameters:
        joint (int): Joint to get the position from.

        Returns:
        float: The current position of the joint in degrees.
        '''
        if Joint.is_valid(joint):
            return self.robots[0].get_joint_position(joint)
        else:
            raise ValueError('Invalid joint ID')

    def get_all_joint_positions(self):
        '''
        Gets the current positions of all joints (in degrees).

        Returns:
        list: A list of current positions in degrees for all joints.
        '''
        return self.robot[0].get_all_joints_positions()
