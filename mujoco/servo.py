class Servo:
    def __init__(self, controller, units_converter, servo_id, pos_limits=[-30,30], vel_limits=[5,50], position=0, velocity=10, reverse=False):
        '''
        Initializes a servo instance. Here, safe values are used as default to avoid hardware problems.

        Parameters:
          controller (Controller): The controller managing this servo.
          units_converter (UnitsConverter): The converter used to transform between application-specific and physical units.
          servo_id (int): The ID of the servo.
          pos_limits (tuple/list): Position limits (min, max) in degrees.
          vel_limits (tuple/list): Velocity limits (min, max) in rpm (a value of 0 rpm means maximum velocity).
          position (int): Home position in degrees.
          velocity (int): Default velocity in rpm.
          load_limit (int): Load to detect an unsafe operation.
          reverse (bool): Whether the servo operates in reverse mode.
        '''
        # Ranges unit conversion for XL430-W250 / XM430-W350 (degrees/rpm)
        self.UC = units_converter
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
        elif pos_limits[0] < self.UC.position.app_range[0] or \
             pos_limits[1] > self.UC.position.app_range[1]:
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

    def get_torque(self) -> float: # or force
        '''
        Get torque/force (%) on the servo.
        '''
        sys_units =  self.controller.get_torque(self.servo_id)
        return self.UC.torque.to_app_units(sys_units)

    def set_velocity(self, velocity: float) -> int:
        '''
        Sets the velocity of the servo (user units).
        '''
        if not (self.vel_limits[0] <= velocity <= self.vel_limits[1]):
            raise ValueError(f'RPM ({velocity}) out of range [{self.vel_limits}] for servo {self.servo_id}.')
        vel_phys = self.UC.velocity.to_sys_units(velocity)
        self.controller.set_velocity(self.servo_id, vel_phys)

    def get_velocity(self) -> float:
        '''
        Gets the current velocity of the servo in RPM.

        Parameters:
        :position (float): Servo position in degrees.
        '''
        vel_phys = self.controller.get_velocity(self.servo_id) # rad/s
        return self.UC.velocity.to_app_units(vel_phys)

    def set_position(self, position: float):
        '''
        Sets the target position of the servo in degrees.

        Parameters:
        :position (float): Servo position in degrees.
        '''
        if not (self.pos_limits[0] <= position <= self.pos_limits[1]):
            raise ValueError(f'Angle ({position}) out of range [{self.pos_limits}] for servo {self.servo_id}')
        factor = -1 if self.reverse else 1
        sys_units = self.UC.position.to_sys_units(factor * position)
        self.controller.set_position(self.servo_id, sys_units)

    def get_position(self) -> float:
        '''
        Gets the current position of the servo in degrees.
        '''
        factor = -1 if self.reverse else 1
        pos_phys = self.controller.get_position(self.servo_id) # rad
        return factor * self.UC.position.to_app_units(pos_phys)

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
