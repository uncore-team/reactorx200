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
        # reactor.enable_all_joints_torques()
        # reactor.home_all_joints()
        time.sleep(5)

        delay = 5
        steps = 20

        joint = Joint.Shoulder
        reactor.enable_joint_torque(joint)
        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        reactor.set_joint_position(joint, -10)
        for _ in range(steps):
            print(f'joint {joint}, torque: {reactor.get_joint_torque(joint)}')
            time.sleep(delay / steps)

        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        reactor.set_joint_position(joint, 10)
        for _ in range(steps):
            print(f'joint {joint}, torque: {reactor.get_joint_torque(joint)}')
            time.sleep(delay / steps)
        deg = reactor.get_joint_position(joint)
        print(f'Current position of joint: {deg:.2f} degrees')

        joint = Joint.Gripper
        reactor.enable_joint_torque(joint)

        reactor.open_gripper()
        for _ in range(steps):
            print(f'joint {joint}, torque: {reactor.get_joint_torque(joint)}')
            time.sleep(delay / steps)

        reactor.close_gripper()
        for _ in range(steps):
            print(f'joint {joint}, torque: {reactor.get_joint_torque(joint)}')
            time.sleep(delay / steps)

        reactor.disable_all_joints_torques()
        reactor.close()

    except Exception as e:
        print(f'Error: {e}')

    finally:
        pass