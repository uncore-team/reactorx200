import numpy as np
import time

from manipulatorarm import ManipulatorArm, Joint
from dynamixelcontroller import DynamixelController
from servo import Servo

class TrossenReactorX200(ManipulatorArm):
# Main class for the physical ReactorX200 robot

    def __init__(self, device_name: str = 'COM5'):
        '''
        Initializes the DXLController class, setting up communication parameters and enabling torque for all servos.

        Parameters:
        port (str): The port name to which the Dynamixel servos are connected (e.g., 'COM5').
        '''
        class Waist(Servo):
            def __init__(self, servo_id=1):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-180, 180]  # pos limits (degrees)
                )

        class Shoulder(Servo):
            def __init__(self, servo_id=2):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-108, 113]
                )

        class ShadowShoulder(Servo):
            def __init__(self, servo_id=3):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-108, 113],
                    reverse_mode=True
                )

        class Elbow(Servo):
            def __init__(self, servo_id=4):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-108, 93]
                )

        class WristAngle(Servo):
            def __init__(self, servo_id=5):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-100, 123]
                )

        class WristRotation(Servo):
            def __init__(self, servo_id=6):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    pos_limits=[-180, 180]
                )

        class Gripper(Servo):
            def __init__(self, servo_id=7):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %
                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000],
                    #pos_limits=[-30, 60]  # -30 -> close, 60 -> open
                    pos_limits=[-20, 50]  # a more conservative range
                )

        super().__init__(
            controller=DynamixelController(device_name),
            joints = {
                Joint.Waist: ( Waist(),  ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Shoulder: ( Shoulder(), ShadowShoulder() ),
                Joint.Elbow: ( Elbow(),  ),
                Joint.WristAngle: ( WristAngle(), ),
                Joint.WristRotation: ( WristRotation(), ),
                Joint.Gripper: ( Gripper(), )
            }
        )


if __name__ == '__main__':

    robot = TrossenReactorX200()

    print('Starting simulation...')

    def print_joint_status(delay:float, steps:int):
        for _ in range(steps):
            velocities = robot.get_joints_velocities()
            positions = robot.get_joints_positions()
            forces = robot.get_joints_forces()
            print(f'velocities: {velocities} positions {positions}, force {forces} %')
            time.sleep(delay/steps)

    def test_joints(speed: float):

        robot.set_joints_velocities([speed] * robot.get_joints_number())
        robot.enable_joints_torques()

        robot.move_joints_to_home()
        print_joint_status(5, 10)

        print(f'\nTesting all joints with velocity {speed} RPM')

        for joint in Joint:
            if joint is Joint.Gripper:
                continue

            for pos in range(-30, 31, 10):
                print(f'Setting joint {joint} to {pos} degrees')
                robot.set_joint_position(joint, pos)
                print_joint_status(3, 10)

            robot.move_joint_to_home(joint)
        # joint = Joint.Shoulder
        # robot.move_joint_to_home(joint)
        # print_joint_status(3, 10)
        # for pos in range(-30, 31, 10):
        #     print(f'Setting joint {joint} to {pos} degrees')
        #     robot.set_joint_position(joint, pos)
        #     print_joint_status(3, 10)

        # robot.move_joint_to_home(joint)

        print('\nOpening gripper...')
        robot.open_gripper()
        print_joint_status(3, 10)
        print('Gripper opened.')

        print('\nClosing gripper...')
        robot.close_gripper()
        print_joint_status(3, 10)
        print('Gripper closed.')

        # robot.disable_joints_torques()

    try:
        test_joints(5) # test with 20 rpm
        # test_joints(50) # test with 50 rpm

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()
