import dynamixel_sdk as dxl
import errors
import numpy as np
import time

from utils import Joint

class Controller:
    def __init__(self, port):
        '''
        Initializes a servo controller instance.

        Parameters:
        port (string): Device name (e.g., COM port or USB port)
        '''
        # Addresses/lengths of relevant control table entries
        # tuple meaning: (command address, number of bytes)
        self.COMMANDS = {
            'DRIVE_MODE':            ('Drive Mode', 10, 1),
            'TORQUE_ENABLE':         ('Torque Enable', 64, 1),
            'HARDWARE_ERROR_STATUS': ('Hardware Error Status', 70, 1),
            'PROFILE_VELOCITY':      ('Profile Velocity', 112, 4),
            'GOAL_POSITION':         ('Goal Position', 116, 4),
            'MOVING_STATUS':         ('Moving Status', 123, 1),
            'PRESENT_LOAD':          ('Present Load', 126, 2),
            'PRESENT_POSITION':      ('Present Position', 132, 4)
        }
        self.PROTOCOL_VERSION = 2  # Protocol version
        self.BAUDRATE = 1000000  # Baudrate
        self.DEVICENAME = port  # Device name (e.g., COM port or USB port)

        # Initialize port and packet handlers
        self.serial = dxl.PortHandler(self.DEVICENAME)
        self.handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        try:
            # Attempt to open the communication port
            if not self.serial.openPort():
                raise Exception(f'Failed to open port {self.DEVICENAME}')

            # Attempt to setup the communication baudrate
            if not self.serial.setBaudRate(self.BAUDRATE):
                raise Exception(f'Failed to set baudrate to {self.BAUDRATE}')

        except Exception as e:
            Exception(f'Initialization error: {str(e)}')
            raise

    def _write_bytes(self, servo, command, value):
        '''
        Writes a byte, word or dword to the specified servo.

        Parameters:
          servo (int): The ID of the servo.
          command (tuple): The address, the command, the number of bytes, and the value to write.

        Returns:
          error: The error returned by the servo (e.g. hardware error)
                 To know the specific hardware error, run the HARDWARE_ERROR_STATUS (70) and,
                 check the value provided by the HARDWARE_ERROR_STATUS (70) command:
                   Bit 5: Overload Error(default). Detects that persistent load that exceeds 
                          maximum output.
                   Bit 4: Electrical Shock Error(default).Detects electric shock on the circuit
                          or insufficient power to operate the motor.
                   Bit 3: Motor Encoder Error. Detects malfunction of the motor encoder	 
                   Bit 2: Overheating Error(default). Detects that internal temperature exceeds 
                          the configured operating temperature.
                   Bit 0: Input Voltage Error. Detects that input voltage exceeds the configured 
                          operating voltage
        '''
        if len(command) != 3:
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Bad command structure.')
        name, address, length = command
        if length == 1:
            result, error = self.handler.write1ByteTxRx(self.serial, servo, address, value)
        elif length == 2:
            result, error = self.handler.write2ByteTxRx(self.serial, servo, address, value)
        elif length == 4:
            result, error = self.handler.write4ByteTxRx(self.serial, servo, address, value)
        else:
            raise Exception(f'Error writing the address "{address}"({name}) for ID {servo}: Bad command length.')

        if (result != dxl.COMM_SUCCESS): # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')

        if (error & errors.HARDWARE):
            print(f'Hardware error for ID {servo}: Please check error status for more info or reboot.')

        return error

    def _read_bytes(self, servo, command):
        '''
        Reads a byte, word or dword to the specified servo.

        Parameters:
        servo (int): The ID of the servo.
        command (tuple): The address, the name of command, and the number of bytes to read.

        Returns:
          value (int): The bytes read from the servo.
          error: The error returned by the servo (e.g. hardware error)
                 To know the specific hardware error, run the HARDWARE_ERROR_STATUS (70) and,
                 check the value provided by the HARDWARE_ERROR_STATUS (70) command:
                   Bit 5: Overload Error(default). Detects that persistent load that exceeds 
                          maximum output.
                   Bit 4: Electrical Shock Error(default).Detects electric shock on the circuit
                          or insufficient power to operate the motor.
                   Bit 3: Motor Encoder Error. Detects malfunction of the motor encoder	 
                   Bit 2: Overheating Error(default). Detects that internal temperature exceeds 
                          the configured operating temperature.
                   Bit 0: Input Voltage Error. Detects that input voltage exceeds the configured 
                          operating voltage
        '''
        if len(command) != 3:
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Bad command structure.')
        name, address, length = command
        if length == 1:
            value, result, error = self.handler.read1ByteTxRx(self.serial, servo, address)
        elif length == 2:
            value, result, error = self.handler.read2ByteTxRx(self.serial, servo, address)
        elif length == 4:
            value, result, error = self.handler.read4ByteTxRx(self.serial, servo, address)
        else:
            raise Exception(f'Error reading the address "{address}"({name}) for ID {servo}: Bad command length.')

        if (result != dxl.COMM_SUCCESS): # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')

        if (error & errors.HARDWARE):
            print(f'Hardware error for ID {servo}: Please check error status for more info or reboot.')

        return value, error

    def _start():
        '''
        '''

    def close(self):
        '''
        Closes the communication port.
        '''
        self.serial.closePort()

    def ping(self, servo):
        '''
        Performs a ping to the specified servo.

        Parameters:
          servo (int): The ID of the servo to ping.

        Returns:
          value (int): The bytes read from the servo.
          error: The error returned by the servo (e.g. hardware error)
        '''
        value, result, error = self.handler.ping(self.serial, servo)
        if (result != dxl.COMM_SUCCESS): # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "PING" for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')
        return value, error

    def factory(self, servo):
        '''
        Performs a factory reset on the specified servo.

        Parameters:
          servo (int): The ID of the servo to reset.

        Returns:
          error: The error returned by the servo (e.g. hardware error)
        '''
        result, error = self.handler.factoryReset(self.serial, servo, 2)  # Reset all except ID and Baudrate
        if (result != dxl.COMM_SUCCESS): # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "FACTORY_RESET" for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')
        return error

    def reboot(self, servo):
        '''
        Reboots the specified servo.

        Parameters:
          servo (int): The ID of the servo to reboot.

        Returns:
          error: The error returned by the servo (e.g. hardware error)
        '''
        result, error = self.handler.reboot(self.serial, servo)
        if (result != dxl.COMM_SUCCESS): # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "REBOOT" for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')
        return error

    def set_position(self, servo: int, position: float):
        '''
        Sets the target position of the servo in position units.

        Parameters:
          position (int): Target position in degrees.
        '''
        error = self._write_bytes(servo, self.COMMANDS['GOAL_POSITION'], position)
        if error:
            raise Exception(f'Error changing the position of the servo {servo}.')

    def get_position(self, servo: int) -> float:
        '''
        Gets the current position of the servo in degrees.

        Returns:
          float: Current position in degrees.
        '''
        pos_units, error = self._read_bytes(servo, self.COMMANDS['PRESENT_POSITION'])
        if error:
            raise Exception(f'Error getting the position of the servo {servo}.')
        return pos_units

    def set_velocity(self, servo: int, velocity: float):
        '''
        Sets the velocity of the servo in RPM.

        Parameters:
          velocity (float): Target velocity in RPM.
        '''
        error = self._write_bytes(servo, self.COMMANDS['PROFILE_VELOCITY'], velocity)
        if error:
            raise Exception(f'Error changing the position of the servo {servo}.')

    def get_velocity(self, servo: int) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Returns:
          float: Current velocity in RPM.
        '''
        vel_units, error = self._read_bytes(servo, self.COMMANDS['PROFILE_VELOCITY'])
        if error:
            raise Exception(f'Error getting the velocity of the servo {servo}.')
        return vel_units

    def enable_torque(self, servo: int) -> float:
        '''
        Enables torque on the servo, allowing it to move.
        '''
        error = self._write_bytes(servo, self.COMMANDS['TORQUE_ENABLE'], 1)
        if error:
            raise Exception(f'Error enabling position control for the servo {servo}.')

    def disable_torque(self, servo: int):
        '''
        Disables torque on the servo, allowing free movement.
        '''
        error = self._write_bytes(servo, self.COMMANDS['TORQUE_ENABLE'], 0)
        if error:
            raise Exception(f'Error disabling position control for the servo {servo}.')

    def get_load(self, servo: int) -> float:
        '''
        Get load percentage on the servo. The abs() of load is used here for the sake of simplicity.
        '''
        load_units, error =  self._read_bytes(servo, self.COMMANDS['PRESENT_LOAD'])
        if error:
            raise Exception(f'Error getting the load rate of the servo {servo}.')
        return abs(load_units)

    def get_status(self, servo: int) -> int:
        '''
        Gets the hardware error status of the servo.
        '''
        status, error =  self._read_bytes(servo, self.COMMANDS['HARDWARE_ERROR_STATUS'])
        if error:
            raise Exception(f'Error getting the status of the servo {servo}.')
        return status

    def get_moving_status(self, servo: int) -> int:
        '''
        '''
        status, error = self._read_bytes(servo, self.COMMANDS['MOVING_STATUS'])
        if error:
            raise Exception(f'Error getting the moving status of the servo {servo}.')
        return status

class FactoryValues:
    def __init__(self, pos_limits_uni, pos_limits_deg, vel_limits_uni, vel_limits_rpm, load_limits_uni, load_limits_rate):
        '''
        Initializes factory-defined limits for Dynamixel servos.

        Parameters:
          pos_limits_uni (list): Position limits in Dynamixel units.
          pos_limits_deg (list): Position limits in degrees.
          vel_limits_uni (list): Velocity limits in Dynamixel units.
          vel_limits_rpm (list): Velocity limits in RPM.
          load_limits_uni (list): Load limits in Dynamixel units.
          load_limits_rate (list): Load limits in percentage.
        '''
        self.pos_limits_uni = pos_limits_uni
        self.pos_limits_deg = pos_limits_deg
        self.vel_limits_uni = vel_limits_uni
        self.vel_limits_rpm = vel_limits_rpm
        self.load_limits_uni = load_limits_uni
        self.load_limits_rate = load_limits_rate

    def rpm_to_vel_units(self, rpm):
        '''
        Converts RPM to Dynamixel velocity units.

        Parameters:
        rpm (float): Velocity in RPM.

        Returns:
        int: Velocity in Dynamixel units.
        '''
        return int(np.interp(rpm, self.vel_limits_rpm, self.vel_limits_uni))

    def vel_units_to_rpm(self, vel_units):
        '''
        Converts Dynamixel velocity units to RPM.

        Parameters:
        vel_units (float): Velocity in Dynamixel units.

        Returns:
        float: Velocity in RPM.
        '''
        return np.interp(vel_units, self.vel_limits_uni, self.vel_limits_rpm)

    def deg_to_pos_units(self, degrees):
        '''
        Converts degrees to Dynamixel position units.

        Parameters:
        degrees (float): Angle in degrees.

        Returns:
        int: Position in Dynamixel units.
        '''
        return int(np.interp(degrees, self.pos_limits_deg, self.pos_limits_uni))

    def pos_units_to_deg(self, pos_units):
        '''
        Converts Dynamixel position units to degrees.

        Parameters:
        pos_units (int): Position in Dynamixel units.

        Returns:
        float: Angle in degrees.
        '''
        return np.interp(pos_units, self.pos_limits_uni, self.pos_limits_deg)

    def load_units_to_percentage(self, pos_units): # two bytes
        '''
        Converts Dynamixel position units to degrees.

        Parameters:
        pos_units (int): Position in Dynamixel units.

        Returns:
        float: Angle in degrees.
        '''
        if ((pos_units >> 15) & 1): # 2-complement?
            pos_units -= 65536 # range -1024 to 1023
        return np.interp(pos_units, self.load_limits_uni, self.load_limits_rate)


class Servo:

    def __init__(self, controller, servo_id, pos_limits=[-30,30], vel_limits=[5,15], position=0, velocity=10, load_limit=15, reverse=False):
        '''
        Initializes a servo instance. Here, safe values are used as default to avoid hardware problems.

        Parameters:
          controller (Controller): The controller managing this servo.
          servo_id (int): The ID of the servo.
          pos_limits (tuple/list): Position limits (min, max) in degrees.
          vel_limits (tuple/list): Velocity limits (min, max) in rpm (a value of 0 rpm means maximum velocity).
          position (int): Home position in degrees.
          velocity (int): Default velocity in rpm.
          load_limit (int): Load to detect an unsafe operation.
          reverse (bool): Whether the servo operates in reverse mode.
        '''
        # Factory-defined limits for XL430-W250 / XM430-W350
        self.FACTORY = FactoryValues(
            pos_limits_uni = [0, 4095],      pos_limits_deg = [-180, 179.91],
            vel_limits_uni = [1, 262],       vel_limits_rpm = [0.229, 61], # A min value of 0 units means infinity speed in mode "Position Control Mode"
            load_limits_uni = [-1000, 1000], load_limits_rate = [-100, 100]
        )

        self.controller = controller
        self.servo_id = servo_id
        self.pos_limits = pos_limits
        self.vel_limits = vel_limits
        self.position = position
        self.velocity = velocity
        self.load_limit = load_limit
        self.reverse = reverse

        _, error = self.get_status()
        if (error & errors.HARDWARE):
            self.reboot()
            raise Exception(f'Hardware error for servo {self.servo_id}. Rebooting... (torque disabled).')

        # Set a safe default velocity (RPM)
        self.set_velocity(self.velocity)

    def change_pos_limits(self, pos_limits: list):
        '''
        '''
        if len(pos_limits) != 2:
            raise ValueError('Wrong range length, please check given values.')
        elif pos_limits[0] < self.FACTORY.pos_limits_deg[0] or \
                pos_limits[1] > self.FACTORY.pos_limits_deg[1]:
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
          > Reverse mode continues enabled if it was previously enabled.
          > It is convenient to check hardware error status or ping before sending new commands.
        '''
        error = self.controller.reboot(self.servo_id)
        if not error:
            time.sleep(2)  # Allow time for reboot
        else:
            raise Exception(f'Error rebooting the servo {self.servo_id}.')

    def factory(self):
        '''
        Reboots the servo configuration settings.
          > Reverse mode is disabled.
          > It is convenient to check hardware error status or ping before sending new commands.
        '''
        error = self.controller.factory(self.servo_id)
        if not error:
            time.sleep(2)  # Allow time for factory reset
        else:
            raise Exception(f'Error making a factory reset the servo {self.servo_id}.')

    def set_position(self, position):
        '''
        Sets the target position of the servo in degrees.

        Parameters:
          position (int): Target position in degrees.
        '''
        if not (self.pos_limits[0] <= position <= self.pos_limits[1]):
            raise ValueError(f'Angle ({position}) out of range [{self.pos_limits}] for servo {self.servo_id}')
        factor = -1 if self.reverse else 1
        pos_units = self.FACTORY.deg_to_pos_units(factor * position)
        self.controller.set_position(self.servo_id, pos_units)

    def get_position(self):
        '''
        Gets the current position of the servo in degrees.

        Returns:
          float: Current position in degrees.
        '''
        pos_units = self.controller.get_position(self.servo_id)
        factor = -1 if self.reverse else 1
        pos_deg = factor * self.FACTORY.pos_units_to_deg(pos_units)
        return pos_deg

    def set_velocity(self, rpm):
        '''
        Sets the velocity of the servo in RPM.

        Parameters:
          velocity (int): Target velocity in RPM.
        '''
        if not (self.vel_limits[0] <= rpm <= self.vel_limits[1]):
            raise ValueError(f'RPM ({rpm}) out of range [{self.vel_limits}] for servo {self.servo_id}')
        vel_units = self.FACTORY.rpm_to_vel_units(rpm)
        self.controller.set_velocity(self.servo_id, vel_units)

    def get_velocity(self):
        '''
        Gets the current velocity of the servo in RPM.

        Returns:
          float: Current velocity in RPM.
        '''
        vel_units = self.controller.get_velocity(self.servo_id)
        vel_rpm = self.FACTORY.vel_units_to_rpm(vel_units)
        return vel_rpm

    def enable_torque(self):
        '''
        Enables torque on the servo, allowing it to move.
        '''
        self.controller.enable_torque(self.servo_id)

    def disable_torque(self):
        '''
        Disables torque on the servo, allowing free movement.
        '''
        self.controller.disable_torque(self.servo_id)

    def get_load(self):
        '''
        Get load percentage on the servo. The abs() of load is used here for the sake of simplicity.
        '''
        load_units =  self.controller.get_load(self.servo_id)
        load_rate = self.FACTORY.load_units_to_percentage(load_units)
        return abs(load_rate)

    def get_status(self) -> int:
        '''
        Gets the hardware error status of the servo.
        '''
        return self.controller.get_status(self.servo_id)

    def get_moving_status(self) -> int:
        '''
        '''
        status = self.controller.get_moving_status(self.servo_id)
        return status

class WaistServo(Servo):
    def __init__(self, controller, servo_id=1):
        super().__init__(controller, servo_id, 
            pos_limits=[-180, 180]  # pos limits (degrees)
        )

class ShoulderServo(Servo):
    def __init__(self, controller, servo_id=2):
        super().__init__(controller, servo_id,
            pos_limits=[-108, 113]
        )

class ShadowShoulderServo(Servo):
    def __init__(self, controller, servo_id=3):
        super().__init__(controller, servo_id, 
            pos_limits=[-108, 113],
            reverse=True
        )

class ElbowServo(Servo):
    def __init__(self, controller, servo_id=4):
        super().__init__(controller, servo_id,
            pos_limits=[-108, 93]
        )

class WristAngleServo(Servo):
    def __init__(self, controller, servo_id=5):
        super().__init__(controller, servo_id,
            pos_limits=[-100, 123]
        )

class WristRotationServo(Servo):
    def __init__(self, controller, servo_id=6):
        super().__init__(controller, servo_id, 
            pos_limits=[-180, 180]
        )

class GripperServo(Servo):
    def __init__(self, controller, servo_id=7):
        super().__init__(controller, servo_id,
            # pos_limits=[-45, 45],  # -45 -> close, 45 -> open
            pos_limits=[-10, 10],  # a more conservative range
            velocity=5
        )

    def open(self):
        return self.set_position(self.pos_limits[1])

    def close(self):
        return self.set_position(self.pos_limits[0])

class ReactorX200:
    def __init__(self, port='COM5'):
        '''
        Initializes the DXLController class, setting up communication parameters and enabling torque for all servos.

        Parameters:
        port (str): The port name to which the Dynamixel servos are connected (e.g., 'COM5').
        '''
        try:
            # Configuration parameters for the Dynamixel servos
            self.controller = Controller(port)

            # Notes: Each joint comprises a list of servos. For instance, the shoulder joint is
            #        composed of two servos: the shoulder and its shadow.
            self.joints = {
                Joint.Waist: ( WaistServo(self.controller),  ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Shoulder: ( ShoulderServo(self.controller), ShadowShoulderServo(self.controller) ),
                Joint.Elbow: ( ElbowServo(self.controller),  ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.WristAngle: ( WristAngleServo(self.controller), ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.WristRotation: ( WristRotationServo(self.controller), ), # tuple of 1 servo (IMPORTANT: comma at the end)
                Joint.Gripper: ( GripperServo(self.controller), ) # tuple of 1 servo (IMPORTANT: comma at the end)
            }
        except Exception as e:
            print(f'Initialization error: {str(e)}')
            raise

    def _get_joint_servos(self, joint:Joint) -> list:
        '''
        Returns the servos associated with a joint.
        '''
        return self.joints[joint]

    def get_joints_number(self) -> int:
        '''
        Returns the servos associated with a joint.
        Returns:
            int: Joint number.
        '''
        return len(self.joints)

    def close(self):
        '''
        Safely closes the connection and disables the joints torque.
        '''
        self.disable_all_joints_torques()
        self.controller.close()

    def home_joint(self, joint: Joint):
        '''
        Moves all joints to their home (default) positions.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.home()
        else:
            raise ValueError('Invalid joint ID')

    def home_joint(self, joint: Joint):
        '''
        Moves a joints to its home (default) position.
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
        Enables position control for the joint given.
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
                raise TypeError(f"Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.")
            servo.open()

    def close_gripper(self):
        '''
        Closes the gripper by moving it to its home position.
        '''
        for servo in self._get_joint_servos(Joint.Gripper):
            if not isinstance(servo, GripperServo):
                raise TypeError(f"Expected joint to be of type GripperServo, but got {type(servo).__name__} instead.")
            servo.close()

    def set_joint_position(self, joint: Joint, position: float):
        '''
        Sets the target position of a specific joint (in degrees).

        Parameters:
        servo (int): Joint to move.
        position (float): The desired position in degrees.
        '''
        if Joint.is_valid(joint):
            for servo in self._get_joint_servos(joint):
                servo.set_position(position)
        else:
            raise ValueError(f'Invalid servo ID: {joint}')

    def set_all_joint_positions(self, positions):
        '''
        Sets positions for all joints (in degrees).

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
        Gets the current position of a specific joint (in degrees).

        Parameters:
        joint (int): Joint to get the position from.

        Returns:
        float: The current position of the joint in degrees.
        '''
        if Joint.is_valid(joint):
            servos = self._get_joint_servos(joint)
            return servos[0].get_position()  # Get position from the first servo
        else:
            raise ValueError(f'Invalid servo ID: {joint}')

    def get_all_joint_positions(self):
        '''
        Gets the current positions of all joints (in degrees).

        Returns:
        list: A list of current positions in degrees for all joints.
        '''
        positions = []
        for joint in self.joints:
            servos = self._get_joint_servos(joint)
            positions.append(
                servos[0].get_position() # only first servo (the shadow is the same)
            )
        return positions

if __name__ == '__main__':
    try:
        reactor = ReactorX200(port='COM5')

        reactor.enable_torque()
        # time.sleep(0.1)
        # reactor.home()
        # reactor.open_gripper()
        # time.sleep(3)
        # reactor.close_gripper()

        joint = Joint.Shoulder
        time.sleep(1)

        deg, error = reactor.get_joint_position(joint)
        print(f'Current position of joint {joint}: {deg:.2f} degrees')

        reactor.set_joint_position(joint, -10)
        time.sleep(1)
        deg, error = reactor.get_joint_position(joint)
        print(f'Current position of joint {joint}: {deg:.2f} degrees')

        reactor.set_joint_position(Joint.Shoulder, 10)
        time.sleep(1)
        deg, error = reactor.get_joint_position(joint)
        print(f'Current position of joint {Joint.Shoulder}: {deg:.2f} degrees')

        #reactor.disable_torque(joint)
        reactor.disable_torque()

        reactor.close()

    except Exception as e:
        print(f'Error: {e}')
