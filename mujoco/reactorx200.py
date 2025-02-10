from enum import Enum
import time

from manipulatorarm import Joint
from mujocoreactorx200 import MuJoCoReactorX200
from trossenreactorx200 import TrossenReactorX200

class ExecutionType(Enum):
    Physical = 0
    Simulated = 1
    DigitalTwin = 2

class ReactorX200:
    def __init__(self, exec_type: ExecutionType = ExecutionType.Simulated, device_name: str = None):
        '''
        Initialize the ReactorX200 class.

        Parameters:
            exec_type (ExecutionType): The type of execution (Physical, Simulated, DigitalTwin).
            device_name (str): COM port for the real robot. If None, the MuJoCo model is used.
        '''
        self.joints_number = 6  # Waist, Shoulder, Elbow, WristAngle, WristRotation, Gripper
        self.exec_type = exec_type
        self.device_name = device_name

        if self.exec_type not in ExecutionType:
            raise ValueError('Wrong execution type.')

        self.robots = []  # list of arms to manage
        if self.exec_type in [ExecutionType.Physical, ExecutionType.DigitalTwin]:  # real
            if self.device_name is None:
                raise ValueError('Wrong device name.')
            self.robots.append(TrossenReactorX200(self.device_name))
        if self.exec_type in [ExecutionType.Simulated, ExecutionType.DigitalTwin]:  # simulated
            self.robots.append(MuJoCoReactorX200())

    def close(self):
        '''
        Closes the connection to all robots.
        '''
        for robot in self.robots:
            robot.close()

    def get_joints_number(self):
        return self.robots[0].get_joints_number()

    def move_joint_to_home(self, joint: Joint):
        '''
        Moves a joint to its home (default) position.

        Parameters:
            joint (Joint): The joint to move to its home position.
        '''
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

        Parameters:
            joint (Joint): The joint to enable torque for.
        '''
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

        Parameters:
            joint (Joint): The joint to disable torque for.
        '''
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

        Parameters:
            joint (Joint): The joint to set torque for.
            value (bool): True to enable torque, False to disable.
        '''
        for robot in self.robots:
            robot.set_joint_torque(joint, value)

    def set_joints_torques(self, values: list[bool]):
        '''
        Enables position control for the servo.

        Parameters:
            values (list[bool]): List of torque values for each joint.
        '''
        for robot in self.robots:
            robot.set_joints_torques(values)

    def get_joint_torque(self, joint: Joint) -> bool:
        '''
        Gets the torque status of the joint.

        Parameters:
            joint (Joint): The joint to get the torque status from.

        Returns:
            bool: True if torque is enabled, False otherwise.
        '''
        return self.robots[0].get_joint_torque(joint)  # first robot

    def get_joints_torques(self) -> list[bool]:
        '''
        Gets the current torques/forces percent of all servos.

        Returns:
            list: A list of current torques/forces for all joints.
        '''
        return self.robots[0].get_joints_torques()  # first robot

    def get_joint_force(self, joint: Joint) -> float:
        '''
        Gets the current torque/force percent of the joint.

        Parameters:
            joint (Joint): Joint to get the torque/force from.

        Returns:
            float: The current torque percent.
        '''
        return self.robots[0].get_joint_force(joint)  # first robot

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

        Parameters:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            rpm (float): Velocity in RPM (0-100).
        '''
        for robot in self.robots:
            robot.set_joint_velocity(joint, rpm)

    def set_joints_velocities(self, velocities: list[float]):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Parameters:
            velocities (list): List of velocities in RPM for each joint.
        '''
        for robot in self.robots:
            robot.set_joints_velocities(velocities)

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Parameters:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        return self.robots[0].get_joint_velocity(joint)  # first robot

    def get_joints_velocities(self) -> list[float]:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        return self.robots[0].get_joints_velocities()  # first robot

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
            joint (Joint): Joint to move.
            position (float): The desired position in degrees.
        '''
        for robot in self.robots:
            robot.set_joint_position(joint, position)

    def set_joints_positions(self, positions: list[float]):
        '''
        Sets positions for all joints (in degrees).

        Parameters:
            positions (list): A list of positions in degrees for each joint.
        '''
        for robot in self.robots:
            robot.set_joints_positions(positions)

    def get_joint_position(self, joint: Joint) -> float:
        '''
        Gets the current position of a specific joint (in degrees).

        Parameters:
            joint (Joint): Joint to get the position from.

        Returns:
            float: The current position of the joint in degrees.
        '''
        return self.robots[0].get_joint_position(joint)  # first robot

    def get_joints_positions(self) -> list[float]:
        '''
        Gets the current positions of all joints (in degrees).

        Returns:
            list: A list of current positions in degrees for all joints.
        '''
        return self.robots[0].get_joints_positions()  # first robot

    def get_joint_position_limits(self, joint: Joint) -> list[float]:
        '''
        Gets the position limits of a specific joint.

        Parameters:
            joint (Joint): Joint to get the position limits of.

        Returns:
            list: A list of position limits for the joint.
        '''
        return self.robots[0].get_joint_position_limits(joint)

    def get_joint_velocity_limits(self, joint: Joint) -> list[float]:
        '''
        Gets the velocity limits of a specific joint.

        Parameters:
            joint (Joint): Joint to get the velocity limits of.

        Returns:
            list: A list of velocity limits for the joint.
        '''
        return self.robots[0].get_joint_velocity_limits(joint)


if __name__ == '__main__':
    robot = ReactorX200(
            device_name='COM5',
            exec_type=ExecutionType.Simulated
        )

    print('Starting simulation...')

    def print_joint_status(joint: Joint, delay: float, steps: int):
        '''
        Prints the status of the specified joint at regular intervals.

        Parameters:
            joint (Joint): The joint to print the status of.
            delay (float): Total delay time in seconds.
            steps (int): Number of steps to divide the delay time into.
        '''
        for _ in range(steps):
            velocity = robot.get_joint_velocity(joint)
            position = robot.get_joint_position(joint)
            force = robot.get_joint_force(joint)
            print(f'{joint}: velocity {velocity:.2f}, position {position:.2f}, force {force:.2f} %')
            time.sleep(delay / steps)

    def test_joints(velocity: float):
        '''
        Tests the movement of all joints at a specified velocity.

        Parameters:
            velocity (float): The velocity in RPM to test the joints with.
        '''
        robot.set_joints_velocities([velocity] * robot.get_joints_number())
        robot.enable_joints_torques()

        print(f'\nTesting all joints with velocity {velocity} RPM')

        for joint in Joint:
            if joint is Joint.Gripper:
                continue

            for pos in range(-30, 31, 10):
                print(f'Setting joint {joint} to {pos} degrees')
                robot.set_joint_position(joint, pos)
                print_joint_status(joint, 3, 10)

            robot.move_joint_to_home(joint)

        # joint = Joint.Shoulder
        # for pos in range(-30, 31, 10):
        #     print(f'Setting joint {joint} to {pos} degrees')
        #     robot.set_joint_position(joint, pos)
        #     print_joint_status(joint, 3, 10)
        # robot.move_joint_to_home(joint)
        # time.sleep(5)

        joint = Joint.Gripper
        pos_limits = robot.get_joint_position_limits(joint)
        print('\nOpening gripper...')
        robot.set_joint_position(joint, pos_limits[1])
        print_joint_status(Joint.Gripper, 3, 10)
        print('Gripper opened.')

        print('\nClosing gripper...')
        robot.set_joint_position(joint, pos_limits[0])
        print_joint_status(joint, 3, 10)
        print('Gripper closed.')

        robot.disable_joints_torques()

    try:
        test_joints(5) # test with 5 rpm
        # test_joints(10) # test with 20 rpm

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()