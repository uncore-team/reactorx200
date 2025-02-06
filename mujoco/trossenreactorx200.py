import numpy as np
import time

from manipulatorarm import ManipulatorArm
from dxlcontroller import DXLController
from servo import Servo
from utils import Joint, UnitsConverter

class DXLServos:
    class Waist(Servo):
        def __init__(self, controller, servo_id=1):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-180, 180]  # pos limits (degrees)
            )

    class Shoulder(Servo):
        def __init__(self, controller, servo_id=2):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-108, 113]
            )

    class ShadowShoulder(Servo):
        def __init__(self, controller, servo_id=3):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-108, 113],
                reverse=True
            )

    class Elbow(Servo):
        def __init__(self, controller, servo_id=4):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-108, 93]
            )

    class WristAngle(Servo):
        def __init__(self, controller, servo_id=5):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-100, 123]
            )

    class WristRotation(Servo):
        def __init__(self, controller, servo_id=6):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                pos_limits=[-180, 180]
            )

    class Gripper(Servo):
        def __init__(self, controller, servo_id=7):
            super().__init__(controller,
                units_converter = UnitsConverter(
                    pos_app_range = [-180, 179.91], # deg
                    vel_app_range = [0.229, 61],    # rpm, a min value of 0 units means infinity speed in mode "Position Control Mode"
                    tor_app_range = [-100, 100],    # %

                    pos_sys_range = [0, 4095],
                    vel_sys_range = [1, 262],
                    tor_sys_range = [-1000, 1000]
                ),
                servo_id=servo_id, 
                #pos_limits=[-30, 60]  # -30 -> close, 60 -> open
                pos_limits=[-10, 10],  # a more conservative range
                velocity=5
            )

        def open(self):
            return self.set_position(self.pos_limits[1])

        def close(self):
            return self.set_position(self.pos_limits[0])

# end DXLServos

# Main class for the physical ReactorX200 robot
class TrossenReactorX200(ManipulatorArm):
    def __init__(self, device_name: str = 'COM5'):
        '''
        Initializes the DXLController class, setting up communication parameters and enabling torque for all servos.

        Parameters:
        port (str): The port name to which the Dynamixel servos are connected (e.g., 'COM5').
        '''
        controller = DXLController(device_name)
        super().__init__(
            controller=controller,
            joints = {
                Joint.Waist: ( DXLServos.Waist(controller),  ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Shoulder: ( DXLServos.Shoulder(controller), DXLServos.ShadowShoulder(controller) ),
                Joint.Elbow: ( DXLServos.Elbow(controller),  ),
                Joint.WristAngle: ( DXLServos.WristAngle(controller), ),
                Joint.WristRotation: ( DXLServos.WristRotation(controller), ),
                Joint.Gripper: ( DXLServos.Gripper(controller), )
            }
        )


if __name__ == '__main__':

    robot = TrossenReactorX200()

    print('Starting simulation...')

    def test_joints(speed: float):

        robot.set_all_joints_velocities([speed] * robot.get_joints_number())
        robot.enable_all_joints_torques()
        robot.home_all_joints()
        time.sleep(10)
        print(f'\nTesting all joints with velocity {speed} RPM')

        # for joint in Joint:
        #     if joint is Joint.Gripper:
        #         continue
        #     for pos in range(-30, 31, 10):
        #         print(f'Setting joint {joint} to {pos} degrees')
        #         robot.set_joint_position(joint, pos)
        #         time.sleep(3)  # wait of some seconds
        #     robot.home_joint(joint)

        print('\nOpening gripper...')
        robot.open_gripper()
        time.sleep(3)
        print('Gripper opened.')

        print('\nClosing gripper...')
        robot.close_gripper()
        time.sleep(3)
        print(f'Torque: {robot.get_joint_torque(Joint.Gripper)}')
        print('Gripper closed.')

        robot.disable_all_joints_torques()

    try:
        test_joints(20) # test with 15 rpm

        test_joints(50) # test with 50 rpm

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()
