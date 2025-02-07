import numpy as np
import time

from manipulatorarm import ManipulatorArm, Joint
from mujococontroller import MuJoCoController
from servo import  Servo

# Main class for the ReactorX200 robot simulator
class MuJoCoReactorX200(ManipulatorArm):
    def __init__(self):
        '''
        Initializes the ReactorX 200 simulator.
        '''
        class Waist(Servo):
            def __init__(self, servo_id=0):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91],  # deg
                    vel_app_range = [0.229, 61],     # rpm
                    tor_app_range = [-100, 100],     # %
                    pos_sys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range = [-8, 8],                             # N/m (see the limits in the .xml file of the arm)
                    pos_limits=[-180, 180]  # pos limits (degrees)
                )

        class Shoulder(Servo):
            def __init__(self, servo_id=1):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91],  # deg
                    vel_app_range = [0.229, 61],     # rpm
                    tor_app_range = [-100, 100],     # %
                    pos_sys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range = [-18, 18],                           # N/m (see the limits in the .xml file of the arm)
                    pos_limits=[-108, 113]
                )

        class Elbow(Servo):
            def __init__(self, servo_id=2):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91],  # deg
                    vel_app_range = [0.229, 61],     # rpm
                    tor_app_range = [-100, 100],     # %
                    pos_sys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range = [-13, 13],                           # N/m (see the limits in the .xml file of the arm)
                    pos_limits=[-108, 93]
                )

        class WristAngle(Servo):
            def __init__(self, servo_id=3):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91],  # deg
                    vel_app_range = [0.229, 61],     # rpm
                    tor_app_range = [-100, 100],     # %
                    pos_sys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range = [-5, 5],                             # N/m (see the limits in the .xml file of the arm)
                    pos_limits=[-100, 123]
                )

        class WristRotation(Servo):
            def __init__(self, servo_id=4):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-180, 179.91],  # deg
                    vel_app_range = [0.229, 61],     # rpm
                    tor_app_range = [-100, 100],     # %
                    pos_sys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range = [-5, 5],                             # N/m (see the limits in the .xml file of the arm)
                    pos_limits=[-180, 180]
                )

        class Gripper(Servo):
            def __init__(self, servo_id=5):
                super().__init__(
                    servo_id=servo_id, 
                    pos_app_range = [-30, 60],    # deg
                    vel_app_range = [0.229, 61],  # rpm
                    tor_app_range = [-100, 100],  # %
                    pos_sys_range = [0.015, 0.035],                             # m
                    vel_sys_range = [0.229*0.015*np.pi/30, 61*0.015*np.pi/30],  # m/s (radius = 1.5 cm)
                    tor_sys_range = [-8, 8],                                    # N (see the limits in the .xml file of the arm)
                    #pos_limits=[-30, 60]  # -30 -> close, 60 -> open
                    pos_limits=[-10, 20] # a more conservative range
                )

        super().__init__(
            controller=MuJoCoController(),
            joints={
                Joint.Waist: ( Waist(), ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Shoulder: ( Shoulder(), ),
                Joint.Elbow: ( Elbow(), ),
                Joint.WristAngle: ( WristAngle(), ),
                Joint.WristRotation: ( WristRotation(), ),
                Joint.Gripper: ( Gripper(), )
            }
        )

if __name__ == '__main__':

    robot = MuJoCoReactorX200()

    print('Starting simulation...')

    def print_joint_status(joint:Joint, delay:float, steps:int):
        for _ in range(steps):
            velocity = robot.get_joint_velocity(joint)
            position = robot.get_joint_position(joint)
            force = robot.get_joint_force(joint)
            print(f'{joint}: velocity {velocity:.2f}, position {position:.2f}, force {force:.2f} %')
            time.sleep(delay/steps)

    def test_joints(speed: float):

        robot.set_joints_velocities([speed] * robot.get_joints_number())
        robot.enable_joints_torques()

        print(f'\nTesting all joints with velocity {speed} RPM')

        for joint in Joint:
            if joint is Joint.Gripper:
                continue

            for pos in range(-30, 31, 10):
                print(f'Setting joint {joint} to {pos} degrees')
                robot.set_joint_position(joint, pos)
                print_joint_status(joint, 3, 10)

            robot.move_joint_to_home(joint)

        print('\nOpening gripper...')
        robot.open_gripper()
        print_joint_status(Joint.Gripper, 3, 10)
        print('Gripper opened.')

        print('\nClosing gripper...')
        robot.close_gripper()
        print_joint_status(Joint.Gripper, 3, 10)
        print('Gripper closed.')

        robot.disable_joints_torques()

    try:
        # test_joints(20) # test with 20 rpm
        test_joints(50) # test with 50 rpm

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()
