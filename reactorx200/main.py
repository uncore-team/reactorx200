from reactorx200 import ReactorX200, ExecutionType
from utils import Joint
from enum import Enum
import errors
import time

if __name__ == '__main__':
    try:
        reactor = ReactorX200(
            port='COM5',
            exec_type=ExecutionType.Simulated
        )
        reactor.enable_all_joints_torques()
        reactor.home_all_joints()
        time.sleep(10)

        joint = Joint.Shoulder
        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        reactor.set_joint_position(joint, -10)
        time.sleep(3)
        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        reactor.set_joint_position(joint, 10)
        time.sleep(3)
        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        reactor.open_gripper()
        time.sleep(3)
        reactor.close_gripper()
        time.sleep(3)
        reactor.home_joint(Joint.Gripper)
        time.sleep(3)

        reactor.disable_all_joints_torques()
        reactor.close()

    except Exception as e:
        print(f'Error: {e}')

    finally:
        pass