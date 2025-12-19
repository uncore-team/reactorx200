import numpy as np
import time

from manipulatorarm import ManipulatorArm, Joint
from dynamixelcontroller import DynamixelController
from servo import Servo

class TrossenReactorX200(ManipulatorArm):
    '''
    Main class for the physical ReactorX200 robot.
    '''
    def __init__(self, device_name: str):
        super().__init__(device_name)

    def _setup(self):
        '''
        Initializes the DXLController class, setting up communication parameters and enabling torque for all servos.

        Parameters:
            device_name (str): The port name to which the Dynamixel servos are connected (e.g., 'COM5').
        '''
        class Waist(Servo):
            def __init__(self, controller, servo_id=1):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-180, 180]  # pos limits (degrees)
                )

        class Shoulder(Servo):
            def __init__(self, controller, servo_id=2):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-108, 113]
                )

        class ShadowShoulder(Servo):
            def __init__(self, controller, servo_id=3):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-108, 113],
                    reverse_mode=True
                )

        class Elbow(Servo):
            def __init__(self, controller, servo_id=4):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-108, 93]
                )

        class WristAngle(Servo):
            def __init__(self, controller, servo_id=5):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-100, 123]
                )

        class WristRotation(Servo):
            def __init__(self, controller, servo_id=6):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    position_limits=[-180, 180]
                )

        class Gripper(Servo):
            def __init__(self, controller, servo_id=7):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity velocity in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    #pos_limits=[-30, 60]  # -30 -> close, 60 -> open
                    position_limits=[-20, 50]  # a more conservative range
                )

        self.controller = DynamixelController(self.device_name)
        self.joints = {
            Joint.Waist: ( Waist(self.controller),  ), # tuple of 1 servo (IMPORTANT: comma at the end)
            Joint.Shoulder: ( Shoulder(self.controller), ShadowShoulder(self.controller) ),
            Joint.Elbow: ( Elbow(self.controller),  ),
            Joint.WristAngle: ( WristAngle(self.controller), ),
            Joint.WristRotation: ( WristRotation(self.controller), ),
            Joint.Gripper: ( Gripper(self.controller), )
        }

if __name__ == '__main__':
    robot = TrossenReactorX200(device_name='COM3')

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

        # joint = Joint.WristRotation
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
