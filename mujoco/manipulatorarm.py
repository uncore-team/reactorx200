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

class ManipulatorArm:
    '''
    Class representing a manipulator arm with multiple joints.
    Provides methods to control and get the status of the joints.
    '''
    def __init__(self, controller:Controller=None, joints:list[tuple[Servo]]=None):
        '''
        Initializes the ManipulatorArm instance.

        Parameters:
            controller (Controller): The controller used to communicate with the servos.
            joints (list): A list of joints in the manipulator arm.
        '''
        self.controller = controller
        self.joints = joints

    def get_joints_number(self) -> int:
        '''
        Get the number of joints of the arm.

        Returns:
            int: Number of joints.
        '''
        return len(self.joints)

    def close(self):
        '''
        Disables the torque of all joints and closes the controller.
        '''
        self.disable_joints_torques()
        self.controller.close()

    def move_joint_to_home(self, joint:Joint):
        '''
        Moves a joint to its home (default) position.

        Parameters:
            joint (Joint): The joint to move to its home position.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        self.set_joint_position(joint, servo.get_home_position())

    def move_joints_to_home(self):
        '''
        Moves all joints to their home (default) positions.
        '''
        for joint in self.joints:
            self.move_joint_to_home(joint)

    def enable_joint_torque(self, joint: Joint):
        '''
        Enables/disables position control for the servo.
        '''
        self.set_joint_torque(joint, True)

    def enable_joints_torques(self):
        '''
        Enables position control for all servos.
        '''
        for joint in self.joints:
            self.set_joint_torque(joint, True)

    def disable_joint_torque(self, joint: Joint):
        '''
        Enables/disables position control for the servo.
        '''
        self.set_joint_torque(joint, False)

    def disable_joints_torques(self):
        '''
        Disables position control for all servos.
        '''
        for joint in self.joints:
            self.set_joint_torque(joint, False)

    def set_joint_torque(self, joint: Joint, value: bool):
        '''
        Enables position control for the servo.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            self.controller.set_torque(servo.get_id(), value)

    def set_joints_torques(self, values: list):
        '''
        Enables position control for the servo.
        '''
        if len(values) != len(self.joints):
            raise ValueError('Number of velocities must match the number of joints.')

        for index, joint in enumerate(self.joints):
            self.set_joint_torque(joint, values[index])

    def get_joint_torque(self, joint: Joint) -> bool:
        '''
        Enables/disables position control for the servo.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        return self.controller.get_torque(servo.get_id())

    def get_joints_torques(self) -> list:
        '''
        Gets the current torques/forces percent of all servos.

        Returns:
            list: A list of current torques/forces for all joints.
        '''
        torques = [
            self.get_joint_torque(joint)
                for joint in self.joints
        ]
        return torques

    def get_joint_force(self, joint: Joint) -> float:
        '''
        Gets the current force of the servo in percentage.

        Parameters:
            joint (Joint): Joint to get the force from.

        Returns:
            float: The current force in percentage.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        return self.controller.get_force(servo.get_id())

    def get_joints_forces(self) -> list:
        '''
        Gets the current forces of all servos in percentage.

        Returns:
            list: A list of current forces in percentage for all joints.
        '''
        forces = [
            self.get_joint_force(joint)
            for joint in self.joints
        ]
        return forces

    def set_joint_velocity(self, joint: Joint, velocity: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Parameters:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            velocity (float): Velocity in RPM (0-100).
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        for servo in self.joints[joint]:
            if not servo.valid_velocity(velocity):
                raise ValueError(f'RPM ({velocity}) out of range [{servo.get_velocity_limits()}] for servo {servo.get_id()}')
            self.controller.set_velocity(servo.get_id(), servo.vel_to_sys(velocity))

    def set_joints_velocities(self, velocities: list):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Parameters:
            velocities (list): List of velocities in RPM for each joint.
        '''
        if len(velocities) != len(self.joints):
            raise ValueError('Number of velocities must match the number of joints.')

        for index, joint in enumerate(self.joints):
            self.set_joint_velocity(joint, velocities[index])

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Parameters:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        if joint not in Joint:
            raise ValueError('Invalid joint ID')

        servo = self.joints[joint][0]  # first joint servo
        velocity = self.controller.get_velocity(servo.get_id())
        return servo.vel_to_app(velocity)

    def get_joints_velocities(self) -> list:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        velocities = [
            self.get_joint_velocity(joint)
            for joint in self.joints
        ]
        return velocities

    def open_gripper(self):
        '''
        Opens the gripper by moving it to its home position.
        '''
        servo = self.joints[Joint.Gripper][0]  # first joint servo
        position = servo.get_position_limits()[1]
        self.set_joint_position(Joint.Gripper, position)

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its home position.
        '''
        servo = self.joints[Joint.Gripper][0]  # first joint servo
        position = servo.get_position_limits()[0]
        self.set_joint_position(Joint.Gripper, position)

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
            joint (Joint): Joint to move.
            position (float): Target position in degrees.
        '''
        if joint not in Joint:
            raise ValueError(f'Invalid joint ID: {joint}')

        for servo in self.joints[joint]:
            if not servo.valid_position(position):
                raise ValueError(f'Position ({position}) out of range [{servo.get_pos_limits()}] for servo {servo.get_id()}')
            factor = -1 if servo.get_reverse_mode() else 1
            self.controller.set_position(servo.get_id(), servo.pos_to_sys(factor * position))

    def set_joints_positions(self, positions: list):
        '''
        Sets the target positions for all joints in degrees.

        Parameters:
            positions (list): A list of positions in degrees for each joint.
        '''
        if len(positions) != len(self.joints):
            raise ValueError('Number of positions must match the number of joints.')

        for index, joint in enumerate(self.joints):
            self.set_joint_position(joint, positions[index])

    def get_joint_position(self, joint: Joint) -> float:
        '''
        Gets the current position of the servo in degrees.

        Parameters:
            joint (Joint): Joint to get the position of.

        Returns:
            float: The current position of the joint in degrees.
        '''
        if joint not in Joint:
            raise ValueError(f'Invalid joint ID: {joint}')

        servo = self.joints[joint][0]   # first joint servo
        position = self.controller.get_position(servo.get_id())  # servo units
        factor = -1 if servo.get_reverse_mode() else 1
        return factor * servo.pos_to_app(position)

    def get_joints_positions(self) -> list:
        '''
        Gets the current positions of all joints in degrees.

        Returns:
            list: A list of current positions in degrees for all joints.
        '''
        positions = [
            self.get_joint_position(joint)
            for joint in self.joints
        ]
        return positions
