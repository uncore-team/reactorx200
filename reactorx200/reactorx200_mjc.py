import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import threading

from utils import Joint

class Controller:
    def __init__(self, robot_name: str, viewer = True):
        '''
        Initializes a servo controller instance.

        Parameters:
        robot_name (string): The name of the robot model.
        '''
        self.robot_name = robot_name
        self.viewer = viewer

        # mujoco_path = os.getenv('MUJOCO_PATH')
        mujoco_path = '.' # debugging
        if not mujoco_path:
            raise EnvironmentError('The "MUJOCO_PATH" environment variable is not set.')

        model_path = os.path.join(mujoco_path, 'model', robot_name, f'{robot_name}.xml')
        if not os.path.exists(model_path):
            raise ValueError(f'Model file not found at: {model_path}')

        try:
            # Load the model and create the simulation context
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)

            # Get keyframe id by name
            keyframe_name = 'home'
            keyframe_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)

            if keyframe_id == -1:
                raise ValueError(f'Keyframe "{keyframe_name}" not found.')

            # Load keyframe in the model state
            mujoco.mj_resetDataKeyframe(self.model, self.data, keyframe_id)

        except Exception as e:
            print(f'Initialization error: {str(e)}')
            raise

        # self._lock = threading.RLock()  # Usar RLock en lugar de Lock
        self._running = threading.Event()
        self._simul_thread = threading.Thread(target=self._simul_loop, name='SimulThread')
        self._viewer_thread = threading.Thread(target=self._viewer_loop, name='ViewerThread')

        self.timestep = self.model.opt.timestep

        self.torque_enabled = [False] * self.model.nu # torque/force status
        self.velocity = [0] * self.model.nu # torque/force status
        self.target_position = [0] * self.model.nu
        self.update_time = 0.2 # sec

        self._start()

    def _start(self):
        '''
        Starts the simulation/visualization in separate threads.
        '''
        if not self._running.is_set():
            self._running.set()
            self._simul_thread.start()
            self._viewer_thread.start()

    def _step(self, servo: int):
        '''
        '''
        if self.torque_enabled[servo]:
            current_pos = self.data.qpos[servo]

            max_step = self.velocity[servo] * self.timestep
            diff = self.target_position[servo] - current_pos
            step = np.clip(diff, -max_step, max_step)

            new_pos = current_pos + step
            self.data.ctrl[servo] = new_pos

    def _simul_loop(self):
        '''
        Main simulation loop.
        '''
        last_time = time.perf_counter()
        while self._running.is_set():
            current_time = time.perf_counter()
            elapsed_time = current_time - last_time

            if elapsed_time >= self.timestep:
                for servo in range(self.model.nu):
                    self._step(servo)
                mujoco.mj_step(self.model, self.data)  # Advance the simulation
                last_time = current_time
            else:
                time.sleep(max(0, min(0.001, self.timestep - elapsed_time)))

    def _viewer_loop(self):
        '''
        Main viewer loop.
        '''
        if self.viewer:
            viewer =  mujoco.viewer.launch_passive(self.model, self.data)
            last_time = time.perf_counter()
            while self._running.is_set() and viewer.is_running():
                current_time = time.perf_counter()
                elapsed_time = current_time - last_time

                if elapsed_time >= self.update_time:
                    viewer.sync()
                    last_time = current_time
                else:
                    time.sleep(max(0, min(0.001, self.update_time - elapsed_time)))  # Short sleep for better responsiveness

    def close(self):
        '''
        Safely kills the threads and disables the joints torque.
        '''
        if self._running.is_set():
            self._running.clear()
            # Wait a while and kill the threads
            time.sleep(0.1)
            if self._simul_thread and self._simul_thread.is_alive():
                self._simul_thread.join(timeout=1.0)

            if self._viewer_thread and self._viewer_thread.is_alive():
                self._viewer_thread.join(timeout=1.0)

    def factory(self, servo: int):
        '''
        Performs a factory reset on the specified servo.

        Parameters:
          servo (int): The ID of the servo to reset.
        '''
        self.torque_enabled[servo] = False

    def reboot(self, servo):
        '''
        Reboots the specified servo.

        Parameters:
          servo (int): The ID of the servo to reboot.
        '''
        self.torque_enabled[servo] = False

    def enable_torque(self, servo: int):
        '''
        Enables torque on the servo, allowing it to move.
        '''
        self.torque_enabled[servo] = True
        self.target_position[servo] = self.data.qpos[servo]

    def disable_torque(self, servo: int):
        '''
        Disables torque on the servo, allowing free movement.
        '''
        self.torque_enabled[servo] = False

    def is_torque_enabled(self, servo: int) -> bool:
        '''
        Get the status of the position control system.
        '''
        return self.torque_enabled[servo]

    def set_velocity(self, servo: int, velocity: float):
        '''
        Sets the velocity of a servo in velocity units (rad/s).

        Parameters:
        :servo (int): The servo id.
        :velocity (float): The velocity in velocity units (rad/).
        '''
        if self.torque_enabled[servo]:
            raise Exception(f'Attempt to set velocity with torque enabled for servo {servo}.')
        self.velocity[servo] = velocity

    def get_velocity(self, servo: int) -> float:
        '''
        Gets the velocity of a servo in velocity units (rad/s).

        Parameters:
        :servo (int): The servo id.

        Returns:
        :float: The velocity in velocity units (rad/s).
        '''
        return self.velocity[servo]

    def set_position(self, servo: int, position: float):
        '''
        Sets the position of a servo in position units (rad).

        Parameters:
        :servo (int): The servo id.
        :position (float): The position in position units (rad).
        '''
        if not self.torque_enabled[servo]:
            raise Exception(f'Attempt to set position with torque disabled for servo {self.servo_id}')
        self.data.qpos[servo] = position

    def get_position(self, servo: int) -> float:
        '''
        Gets the position of a servo in position units (rad).

        Parameters:
        :servo (int): The servo id.

        Returns:
        :float: The position in position units (rad).
        '''
        return self.data.qpos[servo]

    def get_status(self, servo: int) -> int:
        '''
        Gets the hardware error status of the servo.
        '''
        status = 0 # it need to be defined
        return status

    def get_moving_status(self, servo: int) -> int:
        '''
        '''
        status = 0 # it need to be defined
        return status

class UnitsConverter:
    '''
    Stores conversion values and methods for translating between application-specific units and 
    physical units for the servo. We assume here lineal conversions between ranges.
    '''
    class Position:
        '''
        Handles position conversions.
        '''
        def __init__(self, phys_range, user_range):
            self.phys_range = phys_range
            self.user_range = user_range

        def physical_units(self, position: float) -> float:
            '''
            Converts user position units to MuJoCo ones.

            Parameters:
            :position (float): User position units.

            Returns:
            :int: Position in MuJoCo units (rad).
            '''
            return np.interp(position, self.user_range, self.phys_range)

        def user_units(self, position: int) -> float:
            '''
            Converts MuJoCo position units to user ones.

            Parameters:
            units (int): Position in MuJoCo units.

            Returns:
            float: Position in user units.
            '''
            return np.interp(position, self.phys_range, self.user_range)

    class Velocity:
        '''
        Handles velocity conversions.
        '''
        def __init__(self, phys_range, user_range):
            self.phys_range = phys_range
            self.user_range = user_range

        def physical_units(self, velocity: float) -> float:
            '''
            Converts user velocity units to MuJoCo ones.

            Parameters:
            :velocity (float): Velocity in user units.

            Returns:
            :int: Velocity in MuJoCo units.
            '''
            return np.interp(velocity, self.user_range, self.phys_range)

        def user_units(self, velocity: int) -> float:
            '''
            Converts MuJoCo position units to user ones.

            Parameters:
            velocity (int): Velocity in MuJoCo units.

            Returns:
            float: Velocity in user units.
            '''
            return np.interp(velocity, self.phys_range, self.user_range)

    class Torque: # Torque typically, or Force
        '''
        Handles torque/force conversions.
        '''
        def __init__(self, phys_range, user_range):
            self.phys_range = phys_range
            self.user_range = user_range

        def user_units(self, torque: int) -> float: # two bytes
            '''
            Converts MuJoCo torque/force units to user ones.

            Parameters:
            torque (int): Torque/Force in MuJoCo units.

            Returns:
            float: Torque/Force in user units.
            '''
            return np.interp(torque, self.phys_range, self.user_range)

    def __init__(self, pos_phys_range, pos_user_range, vel_phys_range, vel_user_range, tor_phys_range, tor_user_range):
        '''
        Initializes factory-defined limits for MuJoCo servos.

        Parameters:
        pos_phys_range (list): Position limits in MuJoCo units.
        pos_user_range (list): Position limits in user units.
        vel_phys_range (list): Velocity limits in MuJoCo units.
        vel_user_range (list): Velocity limits in user units.
        tor_phys_range (list): Torque/Force limits in MuJoCo units.
        tor_user_range (list): Torque/Force limits in user units.
        '''
        self.position = self.Position(pos_phys_range, pos_user_range)
        self.velocity = self.Velocity(vel_phys_range, vel_user_range)
        self.torque = self.Torque(tor_phys_range, tor_user_range)

class Servo:
    def __init__(self, controller, unit_converter, servo_id, pos_limits=[-30,30], vel_limits=[5,15], torque_limits=[-1,1], position=0, velocity=10, reverse=False):
        '''
        Initializes a servo instance. Here, safe values are used as default to avoid hardware problems.

        Parameters:
        controller (Controller): The controller managing this servo.
        unit_converter (UnitsConverter): The converter used to transform between application-specific and physical units.
        servo_id (int): The ID of the servo.
        pos_limits (tuple/list): Position limits (min, max) in degrees.
        vel_limits (tuple/list): Velocity limits (min, max) in rpm (a value of 0 rpm means maximum velocity).
        torque_limits (tuple/list): Torque limits (min, max) in percentage.
        position (int): Home position in degrees.
        velocity (int): Default velocity in rpm.
        reverse (bool): Whether the servo operates in reverse mode.
        '''
        # Ranges unit conversion for XL430-W250 / XM430-W350 (degrees/rpm)
        self.UC = unit_converter
        self.controller = controller
        self.servo_id = servo_id
        self.pos_limits = pos_limits
        self.vel_limits = vel_limits
        self.position = position # home position
        self.velocity = velocity # initial velocity
        self.reverse = reverse

        self.set_velocity(self.velocity) # before enabling torque

    def change_pos_limits(self, pos_limits: list):
        '''
        '''
        if len(pos_limits) != 2:
            raise ValueError('Wrong range length, please check given values.')
        elif pos_limits[0] < self.UC.position.user_range[0] or \
             pos_limits[1] > self.UC.position.user_range[1]:
            raise ValueError('Given limits are out of factory range, please check given values.')
        self.pos_limits = pos_limits

    def home(self):
        '''
        Moves the servo to its home position if torque is enabled.
        '''
        self.set_position(self.position)

    def reboot(self):
        '''
        Reboots the servo and retains configuration settings.
        Note: No difference between reboot and factory functions in MuJoCo simulations.
        '''
        self.controller.reboot(self.servo_id)

    def factory(self):
        '''
        Reset the servo configuration settings.
        Note: No difference between reboot and factory functions in MuJoCo simulations.
        '''
        self.controller.factory(self.servo_id)

    def enable_torque(self):
        '''
        Enables position control of the servo.
        '''
        self.controller.enable_torque(self.servo_id)

    def disable_torque(self):
        '''
        Disables position control of the servo.
        '''
        self.controller.disable_torque(self.servo_id)

    def set_velocity(self, velocity: float) -> int:
        '''
        Sets the velocity of the servo (user units).
        '''
        if not (self.vel_limits[0] <= velocity <= self.vel_limits[1]):
            raise ValueError(f'RPM ({velocity}) out of range [{self.vel_limits}] for servo {self.servo_id}.')
        vel_phys = self.UC.velocity.physical_units(velocity)
        self.controller.set_velocity(self.servo_id, vel_phys)

    def get_velocity(self) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Parameters:
        :position (float): Servo position in degrees.
        '''
        vel_phys = self.controller.get_velocity(self.servo_id) # rad/s
        return self.UC.velocity.user_units(vel_phys)

    def set_position(self, position: float):
        '''
        Sets the target position of the servo in degrees.

        Parameters:
        :position (float): Servo position in degrees.
        '''
        if not (self.pos_limits[0] <= position <= self.pos_limits[1]):
            raise ValueError(f'Angle ({position}) out of range [{self.pos_limits}] for servo {self.servo_id}')
        factor = -1 if self.reverse else 1
        pos_phys = self.UC.position.physical_units(factor * position)
        self.controller.set_position(self.servo_id, pos_phys)

    def get_position(self) -> float:
        '''
        Gets the current position of the servo in degrees.
        '''
        factor = -1 if self.reverse else 1
        pos_phys = self.controller.get_position(self.servo_id) # rad
        return factor * self.UC.position.user_units(pos_phys)

# Specific servo types for the robot
class WaistServo(Servo):

    def __init__(self, controller, servo_id=0):
        super().__init__(controller, 
            unit_converter=UnitsConverter(
                pos_user_range = [-180, 179.91],  # deg
                vel_user_range = [0.229, 61],     # rpm
                tor_user_range = [-100, 100],     # %

                pos_phys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                vel_phys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                tor_phys_range = [-8, 8]                              # N/m (see the limits in the .xml file of the arm)
            ),
            servo_id=servo_id, 
            pos_limits=[-180, 180]  # pos limits (degrees)
        )

class ShoulderServo(Servo):

    def __init__(self, controller, servo_id=1):
        super().__init__(controller, 
            unit_converter=UnitsConverter(
                pos_user_range = [-180, 179.91],  # deg
                vel_user_range = [0.229, 61],     # rpm
                tor_user_range = [-100, 100],     # %

                pos_phys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                vel_phys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                tor_phys_range = [-18, 18]                            # N/m (see the limits in the .xml file of the arm)
            ),
            servo_id=servo_id, 
            pos_limits=[-108, 113]
        )

class ElbowServo(Servo):
    def __init__(self, controller, servo_id=2):
        super().__init__(controller, 
            unit_converter=UnitsConverter(
                pos_user_range = [-180, 179.91],  # deg
                vel_user_range = [0.229, 61],     # rpm
                tor_user_range = [-100, 100],     # %

                pos_phys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                vel_phys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                tor_phys_range = [-13, 13]                              # N/m (see the limits in the .xml file of the arm)
            ),
            servo_id=servo_id, 
            pos_limits=[-108, 93]
        )

class WristAngleServo(Servo):
    def __init__(self, controller, servo_id=3):
        super().__init__(controller, 
            unit_converter=UnitsConverter(
                pos_user_range = [-180, 179.91],  # deg
                vel_user_range = [0.229, 61],     # rpm
                tor_user_range = [-100, 100],     # %

                pos_phys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                vel_phys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                tor_phys_range = [-5, 5]                              # N/m (see the limits in the .xml file of the arm)
            ),
            servo_id=servo_id, 
            pos_limits=[-100, 123]
        )

class WristRotationServo(Servo):
    def __init__(self, controller, servo_id=4):
        super().__init__(controller, 
            unit_converter=UnitsConverter(
                pos_user_range = [-180, 179.91],  # deg
                vel_user_range = [0.229, 61],     # rpm
                tor_user_range = [-100, 100],     # %

                pos_phys_range = [-180*np.pi/180, 179.91*np.pi/180],  # rad
                vel_phys_range = [0.229*np.pi/30, 61*np.pi/30],       # rad/s
                tor_phys_range = [-5, 5]                              # N/m (see the limits in the .xml file of the arm)
            ),
            servo_id=servo_id, 
            pos_limits=[-180, 180]
        )

class GripperServo(Servo):
    def __init__(self, controller, servo_id=5):
        super().__init__(controller, 
            UnitsConverter(
                pos_user_range = [-30, 60],    # deg
                vel_user_range = [0.229, 61],  # rpm
                tor_user_range = [-100, 100],  # %

                pos_phys_range = [0.015, 0.035],                             # m
                vel_phys_range = [0.229*0.015*np.pi/30, 61*0.015*np.pi/30],  # m/s (radius = 1.5 cm)
                tor_phys_range = [-8, 8]                                     # N (see the limits in the .xml file of the arm)
            ),
            servo_id, 
            #pos_limits=[-45, 45],  # -45 -> close, 45 -> open
            pos_limits=[-10, 10], # a more conservative range
        )

    def open(self):
        return self.set_position(self.pos_limits[1])

    def close(self):
        return self.set_position(self.pos_limits[0])

# Main class for the ReactorX200 robot simulator
class ReactorX200:
    def __init__(self):
        '''
        Initializes the ReactorX 200 simulator.

        Args:
            robot_name (str): Name of the robot (default is 'rx200')
        '''
        self.robot_name = 'reactorx200'

        try:
            # Robot controller configuration
            self.controller = Controller(self.robot_name)

            # Initialization of joints (each joint has a list of servos)
            self.joints = {
                Joint.Waist: ( WaistServo(self.controller), ),  # Waist servo
                Joint.Shoulder: ( ShoulderServo(self.controller), ), # Shoulder servo
                Joint.Elbow: ( ElbowServo(self.controller), ),   # Elbow servo
                Joint.WristAngle: ( WristAngleServo(self.controller), ), # Wrist angle servo
                Joint.WristRotation: ( WristRotationServo(self.controller), ), # Wrist rotation servo
                Joint.Gripper: ( GripperServo(self.controller), ) # Gripper servo
            }
        except Exception as e:
            print(f'Initialization error: {str(e)}')
            raise        


    def _get_joint_servos(self, joint: Joint) -> list:
        '''
        Returns the servos associated with a joint.
        '''
        return self.joints[joint]

    def get_joints_number(self) -> int:
        '''
        Get the number of joints of the arm.
        Returns:
            int: Number of joints.
        '''
        return len(self.joints)

    def close(self):
        self.disable_all_joints_torques()
        self.controller.close()

    def home_joint(self, joint: Joint):
        '''
        Moves a joint to its home (default) position.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.home()
        else:
            raise ValueError('Invalid joint ID')

    def home_all_joints(self):
        '''
        Moves all joints to their home (default) positions.
        '''
        for joint in self.joints:
            for servo in self._get_joint_servos(joint):
                servo.home()

    def enable_joint_torque(self, joint: Joint):
        '''
        Enables position control for the servo.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.enable_torque()
        else:
            raise ValueError('Invalid joint ID')

    def enable_all_joints_torques(self):
        '''
        Enables position control for all servos.
        '''
        for joint in self.joints:
            for servo in self._get_joint_servos(joint):
                servo.enable_torque()

    def disable_joint_torque(self, joint: Joint):
        '''
        Disables position control for the servo.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.disable_torque()                
        else:
            raise ValueError('Invalid joint ID')

    def disable_all_joints_torques(self):
        '''
        Disables position control for all servos.
        '''
        for joint in self.joints:
            for servo in self._get_joint_servos(joint):
                servo.disable_torque()

    def open_gripper(self):
        '''
        Opens the gripper by moving it to its home position.
        '''
        for servo in self._get_joint_servos(Joint.Gripper):
            if not isinstance(servo, GripperServo):
                raise TypeError(f'Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.')
            servo.open()

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its home position.
        '''
        for servo in self._get_joint_servos(Joint.Gripper):
            if not isinstance(servo, GripperServo):
                raise TypeError(f'Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.')
            servo.close()

    def set_joint_velocity(self, joint: Joint, rpm: float):
        '''
        Sets the maximum velocity of the servo in RPM.
        This limits the rate of change of the target position.

        Args:
            joint (Joint): Joint to control (e.g., Shoulder, Elbow).
            rpm (float): Velocity in RPM (0-100).
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.set_velocity(rpm)
        else:
            raise ValueError('Invalid joint ID')

    def set_all_joints_velocities(self, velocities: list):
        '''
        Sets the maximum velocities for all servos in RPM.
        This limits the rate of change of the target positions for all joints.

        Args:
            velocities (list): List of velocities in RPM for each joint.
        '''
        if len(velocities) != len(self.joints):
            raise ValueError('Number of velocities must match the number of joints.')

        for index, joint in enumerate(self.joints):
            for servo in self._get_joint_servos(joint):
                servo.set_velocity(velocities[index])

    def get_joint_velocity(self, joint: Joint) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Args:
            joint (Joint): Joint to get the velocity from.

        Returns:
            float: The current velocity in RPM.
        '''
        if Joint.is_valid(joint):
            servos = self._get_joint_servos(joint)
            return servos[0].get_velocity()  # Get velocity from the first servo
        else:
            raise ValueError('Invalid joint ID')

    def get_all_joint_velocities(self) -> list:
        '''
        Gets the current velocities of all servos in RPM.

        Returns:
            list: A list of current velocities in RPM for all joints.
        '''
        velocities = []
        for joint in self.joints:
            servos = self._get_joint_servos(joint)
            velocities.append(
                servos[0].get_velocity()  # Get velocity from the first servo
            )
        return velocities

    def open_gripper(self):
        '''
        Opens the gripper by moving it to its home position.
        '''
        for servo in self._get_joint_servos(Joint.Gripper):
            if not isinstance(servo, GripperServo):
                raise TypeError(f'Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.')
            servo.open()

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its home position.
        '''
        for servo in self._get_joint_servos(Joint.Gripper):
            if not isinstance(servo, GripperServo):
                raise TypeError(f'Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.')
            servo.close()

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
            joint (Joint): Joint to move.
            position (float): Target position in degrees.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.set_position(position)
        else:
            raise ValueError(f'Invalid joint ID: {joint}')

    def set_all_joint_positions(self, positions):
        '''
        Sets the target positions for all joints in degrees.

        Parameters:
        positions (list): A list of positions in degrees for each joint.
        '''
        if len(positions) != len(self.joints):
            raise ValueError('Number of positions must match the number of joints.')

        for index, joint in enumerate(self.joints):
            for servo in self._get_joint_servos(joint):
                servo.set_position(positions[index])

    def get_joint_position(self, joint: Joint) -> float:
        '''
        Gets the current position of the servo in degrees.

        Args:
            joint (Joint): Joint to get the position from.

        Returns:
            float: The current position of the joint in degrees.
        '''
        if Joint.is_valid(joint):
            servos = self._get_joint_servos(joint)
            return servos[0].get_position()  # Get position from the first servo
        else:
            raise ValueError(f'Invalid joint ID: {joint}')

    def get_all_joint_positions(self):
        '''
        Gets the current positions of all joints in degrees.

        Returns:
        list: A list of current positions in degrees for all joints.
        '''
        positions = []
        for joint in self.joints:
            servos = self._get_joint_servos(joint)
            positions.append(
                servos[0].get_position()  # Get position from the first servo
            )
        return positions

if __name__ == '__main__':

    robot = ReactorX200()

    print('Starting simulation...')

    def index_to_joint(index: int) -> Joint:
        if 0 <= index <= len(self.joints):
            return Joint(index)
        print('Bad joint index')
        return None

    def test_joints(speed: float):

        robot.set_all_joints_velocities([speed]*len(self.joints))
        robot.enable_all_joints_torques()

        # Establecer velocidad
        print(f'\nTesting all joints with velocity {speed} RPM')
        # for joint in Joint:
        #     if joint is Joint.Gripper:
        #         continue
        #     for pos in range(-30, 31, 10):  # Probar posiciones de -30 a 30 grados
        #         print(f'Setting joint {joint} to {pos} degrees')
        #         robot.set_joint_position(joint, pos)
        #         time.sleep(5)  # Pausa de 2 segundos después de cambiar la posición
        # joint = Joint.Shoulder
        # for pos in range(-30, 31, 10):  # Probar posiciones de -30 a 30 grados
        #     print(f'Setting joint {joint} to {pos} degrees')
        #     robot.set_joint_position(joint, pos)
        #     time.sleep(5)  # Pausa de 2 segundos después de cambiar la posición

        # Probar pinza (abrir y cerrar)
        print('\nOpening gripper...')
        robot.open_gripper()
        time.sleep(2)  # Pausa de 2 segundos después de abrir la pinza
        print('Gripper opened.')
        print('\nClosing gripper...')
        robot.close_gripper()
        time.sleep(2)  # Pausa de 2 segundos después de cerrar la pinza
        print('Gripper closed.')

        robot.disable_all_joints_torques()

    try:
        # Realizar las pruebas con 15 rpm
        test_joints(15)

        # Realizar las pruebas con 45 rpm
        test_joints(45)

    except Exception as e:
        print(f'Error: {e}')

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')

    finally:
        print('Simulation finished.')
        robot.close()
