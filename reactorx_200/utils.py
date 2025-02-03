from enum import Enum

# Enum to represent the different joints of the robot
class Joint(Enum):
    Waist = 0
    Shoulder = 1
    Elbow = 2
    WristAngle = 3
    WristRotation = 4
    Gripper = 5

    @classmethod
    def is_valid(self, joint: 'Joint') -> bool:
        return joint in self