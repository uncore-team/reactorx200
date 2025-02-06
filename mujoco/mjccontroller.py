import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import threading

class MJCController:
    def __init__(self, robot_name: str='reactorx200', show_viewer = True):
        '''
        Initializes a servo controller instance.

        Parameters:
        robot_name (string): The name of the robot model.
        '''
        self.robot_name = robot_name
        self.show_viewer = show_viewer

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

        self.lock = threading.RLock()
        self.running = threading.Event()
        self.simul_thread = threading.Thread(target=self._simul_loop, name='Simulation Thread')
        self.viewer_thread = threading.Thread(target=self._viewer_loop, name='Viewer Thread')

        self.update_time = 0.2 # sec
        self.timestep = self.model.opt.timestep # MuJoCo sample time

        self.torque_enabled = [False] * self.model.nu # torque/force status
        self.target_velocity = [0] * self.model.nu # torque/force status
        self.target_position = [0] * self.model.nu

        self._start()

    def _start(self):
        '''
        Starts the simulation/visualization in separate threads.
        '''
        if not self.running.is_set():
            self.running.set() # lets work
            self.simul_thread.start()
            self.viewer_thread.start()

    def _step(self):
        '''
        '''
        with self.lock:
            for servo in range(self.model.nu):
                if self.torque_enabled[servo]:
                    current_pos = self.data.qpos[servo]
                    max_step = self.target_velocity[servo] * self.timestep

                    diff = self.target_position[servo] - current_pos
                    step = np.clip(diff, -max_step, max_step)

                    self.data.ctrl[servo] = current_pos + step # new position

            mujoco.mj_step(self.model, self.data)  # Advance the simulation

    def _simul_loop(self):
        '''
        Main simulation loop.
        '''
        last_time = time.perf_counter()
        while self.running.is_set():
            current_time = time.perf_counter()
            elapsed_time = current_time - last_time

            if elapsed_time >= self.timestep:
                self._step()
                last_time = current_time
            else:
                time.sleep(max(0, min(0.001, self.timestep - elapsed_time)))

    def _viewer_loop(self):
        '''
        Main viewer loop.
        '''
        if self.show_viewer:
            viewer =  mujoco.viewer.launch_passive(self.model, self.data)
            last_time = time.perf_counter()
            while self.running.is_set() and viewer.is_running():
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
        if self.running.is_set():
            self.running.clear()
            # Wait a while and kill the threads
            time.sleep(0.1)
            if self.simul_thread and self.simul_thread.is_alive():
                self.simul_thread.join(timeout=1.0)

            if self.viewer_thread and self.viewer_thread.is_alive():
                self.viewer_thread.join(timeout=1.0)

    def factory(self, servo: int):
        '''
        Performs a factory reset on the specified servo.

        Parameters:
          servo (int): The ID of the servo to reset.
        '''
        with self.lock:
            self.torque_enabled[servo] = False

    def reboot(self, servo):
        '''
        Reboots the specified servo.

        Parameters:
          servo (int): The ID of the servo to reboot.
        '''
        with self.lock:
            self.torque_enabled[servo] = False

    def enable_torque(self, servo: int):
        '''
        Enables torque on the servo, allowing it to move.
        '''
        with self.lock:
            self.torque_enabled[servo] = True
            self.target_position[servo] = self.data.qpos[servo]

    def disable_torque(self, servo: int):
        '''
        Disables torque on the servo, allowing free movement.
        '''
        with self.lock:
            self.torque_enabled[servo] = False

    def is_torque_enabled(self, servo: int) -> bool:
        '''
        Get the status of the position control system.
        '''
        with self.lock:
            return self.torque_enabled[servo]

    def get_torque(self, servo: int) -> float:
        '''
        Gets the torque/force of a servo in torque units (N/m or N).

        Parameters:
        :servo (int): The servo id.

        Returns:
        :float: The torque/force in torque units (N/m or N).
        '''
        with self.lock:
            if not self.torque_enabled[servo]:
                raise Exception(f'Attempt to get torque percent with the position control disabled for servo {servo}')
            joint = self.model.actuator_trnid[servo, 0] 
            axes = self.model.jnt_axis[joint]                 # [1 0 0], [0 1 0] or [0 0 1]
            torques = self.data.sensordata[joint*3:joint*3+3] # [torque X, torque Y, torque Z]
            return np.dot(axes, torques)

    def set_velocity(self, servo: int, velocity: float):
        '''
        Sets the velocity of a servo in velocity units (rad/s).

        Parameters:
        :servo (int): The servo id.
        :velocity (float): The velocity in velocity units (rad/).
        '''
        with self.lock:
            if self.torque_enabled[servo]:
                raise Exception(f'Attempt to set velocity with torque enabled for servo {servo}.')
            self.target_velocity[servo] = velocity

    def get_velocity(self, servo: int) -> float:
        '''
        Gets the velocity of a servo in velocity units (rad/s).

        Parameters:
        :servo (int): The servo id.

        Returns:
        :float: The velocity in velocity units (rad/s).
        '''
        with self.lock:
            return self.target_velocity[servo]

    def set_position(self, servo: int, position: float):
        '''
        Sets the position of a servo in position units (rad).

        Parameters:
        :servo (int): The servo id.
        :position (float): The position in position units (rad).
        '''
        with self.lock:
            if not self.torque_enabled[servo]:
                raise Exception(f'Attempt to set position with torque disabled for servo {servo}')
            self.target_position[servo] = position

    def get_position(self, servo: int) -> float:
        '''
        Gets the position of a servo in position units (rad).

        Parameters:
        :servo (int): The servo id.

        Returns:
        :float: The position in position units (rad).
        '''
        with self.lock:
            return self.data.qpos[servo]

    def get_status(self, servo: int) -> int:
        '''
        Gets the hardware error status of the servo.
        '''
        with self.lock:
            status = 0 # it need to be defined
            return status

    def get_moving_status(self, servo: int) -> int:
        '''
        '''
        with self.lock:
            status = 0 # it need to be defined
            return status
