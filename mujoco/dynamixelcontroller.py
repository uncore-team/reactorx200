from controller import Controller
import dynamixel_sdk as dxl
import errors
import time

class DynamixelController(Controller):
    def __init__(self, device_name: str, baud_rate: int = 1000000, protocol: int = 2):
        '''
        Initializes a DynamixelController instance.

        Parameters:
        device_name (str): Device name (e.g., COM port or USB port)
        baud_rate (int): Baudrate for communication (default is 1000000)
        protocol (int): Protocol version (default is 2)
        '''
        self.device_name = device_name  # device name (e.g., COM port or USB port)
        self.baud_rate = baud_rate  # baudrate
        self.protocol = protocol  # protocol version

        # Addresses/lengths of relevant control table entries
        # tuple meaning: (command name, command address, number of bytes)
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

        # Initialize port and packet handlers
        self.serial = dxl.PortHandler(self.device_name)
        self.handler = dxl.PacketHandler(self.protocol)

        self._start()

    def _write_bytes(self, servo: int, command: tuple, value: int) -> int:
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
        name, address, length = command
        if len(command) != 3:
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Bad command structure.')
        if length == 1:
            result, error = self.handler.write1ByteTxRx(self.serial, servo, address, value)
        elif length == 2:
            result, error = self.handler.write2ByteTxRx(self.serial, servo, address, value)
        elif length == 4:
            result, error = self.handler.write4ByteTxRx(self.serial, servo, address, value)
        else:
            raise Exception(f'Error writing the address "{address}"({name}) for ID {servo}: Bad command length.')

        if result != dxl.COMM_SUCCESS:  # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Communication error {self.handler.getTxRxResult(result)}')

        if error & errors.HARDWARE:
            print(f'Hardware error for ID {servo}: Please check error status for more info or reboot.')

        return error

    def _read_bytes(self, servo: int, command: tuple) -> int:
        '''
        Reads a byte, word or dword from the specified servo.

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
        name, address, length = command
        if len(command) != 3:
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Bad command structure.')
        if length == 1:
            value, result, error = self.handler.read1ByteTxRx(self.serial, servo, address)
        elif length == 2:
            value, result, error = self.handler.read2ByteTxRx(self.serial, servo, address)
        elif length == 4:
            value, result, error = self.handler.read4ByteTxRx(self.serial, servo, address)
        else:
            raise Exception(f'Error reading the address "{address}"({name}) for ID {servo}: Bad command length.')

        if result != dxl.COMM_SUCCESS:  # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "{name}"({address}) for ID {servo}: Communication error\n{self.handler.getTxRxResult(result)}')

        if error & errors.HARDWARE:
            print(f'Hardware error for ID {servo}: Please check error status for more info or reboot.')

        return value, error

    def _start(self):
        '''
        Starts the communication with the servos by opening the port and setting the baudrate.
        Scans for connected servos and reboots them if necessary.
        '''
        try:
            # Attempt to open the communication port
            if not self.serial.openPort():
                raise Exception(f'Failed to open port {self.device_name}')

            # Attempt to setup the communication baudrate
            if not self.serial.setBaudRate(self.baud_rate):
                raise Exception(f'Failed to set baudrate to {self.baud_rate}')

            # Scan for servos with IDs from 1 to 253
            print(f'Scanning for connected servos...')
            for servo in range(1, 8):  # from 1 to 8
                print(f'\tping to ID {servo}')
                _, result, error = self.handler.ping(self.serial, servo)
                if result != dxl.COMM_SUCCESS:
                    print(f"Error on ID {servo}: {self.handler.getTxRxResult(result)}")

                if (error & errors.HARDWARE):
                    print(f'Rebooting for recovering ID {servo}...')
                    self.reboot(servo)

        except Exception as e:
            raise Exception(f'Initialization error: {str(e)}')

    def close(self):
        '''
        Closes the communication port.
        '''
        self.serial.closePort()

    def factory(self, servo: int):
        '''
        Performs a factory reset on the specified servo.
          > Reverse mode is disabled.
          > It is convenient to check hardware error status or ping before sending new commands.

        Parameters:
          servo (int): The ID of the servo to reset.

        Returns:
          error: The error returned by the servo (e.g. hardware error)
        '''
        result, error = self.handler.factoryReset(self.serial, servo, 2)  # Reset all except ID and baudrate
        if result != dxl.COMM_SUCCESS:  # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "FACTORY_RESET" for ID {servo}: Communication error\n{self.handler.getTxRxResult(result)}')
        if error != errors.NO_ERROR:
            raise Exception(f'Error running the command "FACTORY_RESET" for ID {servo}: Communication error\n{self.handler.getRxPacketError(error)}')
        time.sleep(2)  # Allow time for reboot

    def reboot(self, servo: int):
        '''
        Reboots the specified servo.
          > Reverse mode continues enabled if it was previously enabled.
          > It is convenient to check hardware error status or ping before sending new commands.

        Parameters:
          servo (int): The ID of the servo to reboot.

        Returns:
          error: The error returned by the servo (e.g. hardware error)
        '''
        result, error = self.handler.reboot(self.serial, servo)
        if result != dxl.COMM_SUCCESS:  # 0, -1000, -2000, -1001, -9000, -3001, -3002
            raise Exception(f'Error running the command "REBOOT" for ID {servo}: Communication error\n{self.handler.getTxRxResult(result)}')
        if error != errors.NO_ERROR:
            raise Exception(f'Error running the command "REBOOT" for ID {servo}: Communication error\n{self.handler.getRxPacketError(error)}')
        time.sleep(2)  # Allow time for reboot

    def set_torque(self, servo: int, value: bool):
        '''
        Enables or disables torque on the servo, allowing it to move or not.

        Parameters:
          servo (int): The ID of the servo.
          value (bool): True to enable torque, False to disable.
        '''
        error = self._write_bytes(servo, self.COMMANDS['TORQUE_ENABLE'], 1 if value else 0)
        if error:
            raise Exception(f'Error enabling position control for the servo {servo}.')

    def get_torque(self, servo: int) -> bool:
        '''
        Gets the status of the position control system.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          bool: True if torque is enabled, False otherwise.
        '''
        value, error = self._read_bytes(servo, self.COMMANDS['TORQUE_ENABLE'])
        if error:
            raise Exception(f'Error reading the status of the position control for the servo {servo}.')
        return True if value else False

    def get_force(self, servo: int) -> float:
        '''
        Gets load percentage on the servo.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          float: The current load percentage on the servo.
        '''
        tor_units, error = self._read_bytes(servo, self.COMMANDS['PRESENT_LOAD'])
        if error:
            raise Exception(f'Error getting the load rate of the servo {servo}.')
        if (tor_units >> 15) & 1:  # 2-complement?
            tor_units -= 65536  # range -1024 to 1023
        return tor_units

    def set_velocity(self, servo: int, velocity: float):
        '''
        Sets the velocity of the servo in velocity units.

        Parameters:
          servo (int): The ID of the servo.
          velocity (float): The velocity in velocity units.
        '''
        error = self._write_bytes(servo, self.COMMANDS['PROFILE_VELOCITY'], int(velocity))
        if error:
            raise Exception(f'Error changing the velocity of the servo {servo}.')

    def get_velocity(self, servo: int) -> float:
        '''
        Gets the current velocity of the servo in velocity units.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          float: The current velocity of the servo in velocity units.
        '''
        vel_units, error = self._read_bytes(servo, self.COMMANDS['PROFILE_VELOCITY'])
        if error:
            raise Exception(f'Error getting the velocity of the servo {servo}.')
        return vel_units

    def set_position(self, servo: int, position: float):
        '''
        Sets the target position of the servo in position units.

        Parameters:
          servo (int): The ID of the servo.
          position (float): The position in position units.
        '''
        error = self._write_bytes(servo, self.COMMANDS['GOAL_POSITION'], int(position))
        if error:
            raise Exception(f'Error changing the position of the servo {servo}.')

    def get_position(self, servo: int) -> float:
        '''
        Gets the current position of the servo in position units.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          float: The current position of the servo in position units.
        '''
        pos_units, error = self._read_bytes(servo, self.COMMANDS['PRESENT_POSITION'])
        if error:
            raise Exception(f'Error getting the position of the servo {servo}.')
        return pos_units

    def get_status(self, servo: int) -> int:
        '''
        Gets the hardware error status of the servo.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          int: The hardware error status of the servo.
        '''
        status, error = self._read_bytes(servo, self.COMMANDS['HARDWARE_ERROR_STATUS'])
        if error:
            raise Exception(f'Error getting the status of the servo {servo}.')
        return status

    def get_moving_status(self, servo: int) -> int:
        '''
        Gets the moving status of the servo.

        Parameters:
          servo (int): The ID of the servo.

        Returns:
          int: The moving status of the servo.
        '''
        status, error = self._read_bytes(servo, self.COMMANDS['MOVING_STATUS'])
        if error:
            raise Exception(f'Error getting the moving status of the servo {servo}.')
        return status
