import numpy as np
import time

from manipulatorarm import ManipulatorArm, Joint
from mujococontroller import MuJoCoController
from servo import Servo

class MuJoCoReactorX200(ManipulatorArm):
    '''
    Main class for the ReactorX200 robot simulator.
    '''
    def __init__(self, device_name: str='reactorx200'):
        super().__init__(device_name)

    def _setup(self):
        '''
        Setup the simulated ReactorX 200.
        '''
        class Waist(Servo):
            def __init__(self, controller, servo_id=0):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-180, 179.91],  # deg
                    vel_app_range=[0.229, 61],     # rpm
                    tor_app_range=[-100, 100],     # %
                    pos_sys_range=[-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range=[0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range=[-8, 8],                             # N/m (see the limits in the .xml file of the arm)
                    position_limits=[-180, 180]  # pos limits (degrees)
                )

        class Shoulder(Servo):
            def __init__(self, controller, servo_id=1):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-180, 179.91],  # deg
                    vel_app_range=[0.229, 61],     # rpm
                    tor_app_range=[-100, 100],     # %
                    pos_sys_range=[-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range=[0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range=[-18, 18],                           # N/m (see the limits in the .xml file of the arm)
                    position_limits=[-108, 113]
                )

        class Elbow(Servo):
            def __init__(self, controller, servo_id=2):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-180, 179.91],  # deg
                    vel_app_range=[0.229, 61],     # rpm
                    tor_app_range=[-100, 100],     # %
                    pos_sys_range=[-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range=[0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range=[-13, 13],                           # N/m (see the limits in the .xml file of the arm)
                    position_limits=[-108, 93]
                )

        class WristAngle(Servo):
            def __init__(self, controller, servo_id=3):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-180, 179.91],  # deg
                    vel_app_range=[0.229, 61],     # rpm
                    tor_app_range=[-100, 100],     # %
                    pos_sys_range=[-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range=[0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range=[-5, 5],                             # N/m (see the limits in the .xml file of the arm)
                    position_limits=[-100, 123]
                )

        class WristRotation(Servo):
            def __init__(self, controller, servo_id=4):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-180, 179.91],  # deg
                    vel_app_range=[0.229, 61],     # rpm
                    tor_app_range=[-100, 100],     # %
                    pos_sys_range=[-180*np.pi/180, 179.91*np.pi/180],  # rad
                    vel_sys_range=[0.229*np.pi/30, 61*np.pi/30],       # rad/s
                    tor_sys_range=[-5, 5],                             # N/m (see the limits in the .xml file of the arm)
                    position_limits=[-180, 180]
                )

        class Gripper(Servo):
            def __init__(self, controller, servo_id=5):
                super().__init__(
                    controller=controller,
                    servo_id=servo_id, 
                    pos_app_range=[-30, 60],    # deg
                    vel_app_range=[0.229, 61],  # rpm
                    tor_app_range=[-100, 100],  # %
                    pos_sys_range=[0.015, 0.035],                             # m
                    vel_sys_range=[0.229*0.015*np.pi/30, 61*0.015*np.pi/30],  # m/s (radius = 1.5 cm)
                    tor_sys_range=[-8, 8],                                    # N (see the limits in the .xml file of the arm)
                    position_limits=[-30, 60]  # -30 -> close, 60 -> open
                )

        self.controller = MuJoCoController(self.device_name)
        self.joints = {
                Joint.Waist: ( Waist(self.controller), ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Shoulder: ( Shoulder(self.controller), ),
                Joint.Elbow: ( Elbow(self.controller), ),
                Joint.WristAngle: ( WristAngle(self.controller), ),
                Joint.WristRotation: ( WristRotation(self.controller), ),
                Joint.Gripper: ( Gripper(self.controller), )
        }

if __name__ == '__main__':
    robot = MuJoCoReactorX200()

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
        #test_joints(10) # test with 10 rpm
        test_joints(20) # test with 20 rpm

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()
