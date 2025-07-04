# Modified from https://github.com/jimmyyhwu/tidybot2
import os
os.environ['CTR_TARGET'] = 'Hardware'  # pylint: disable=wrong-import-position

import math
import queue
import threading
import time
from enum import Enum
import atexit
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
from threadpoolctl import threadpool_limits

from hex_vehicle import PublicAPI as VehicleAPI
from constants import h_x, h_y
from constants import POLICY_CONTROL_PERIOD
from utils import create_pid_file


# Vehicle
CONTROL_FREQ = 200
CONTROL_PERIOD = 1.0 / CONTROL_FREQ  # 5 ms
NUM_CASTERS = 4
MOTOR_MAPPING = [7, 6, 1, 0, 3, 2, 5, 4]
DIRECTION_ARRAY = [-1, 1, -1, 1, -1, 1, -1, 1]


# Caster
b_x = 0.020  # Caster offset (m)
b_y = 0.0    # Lateral caster offset (m)
r = 0.06135  # Wheel radius (m)
N_s = 1      # Steer gear ratio
N_r1 = 1     # Drive gear ratio (1st stage)
N_r2 = 1     # Drive gear ratio (2nd stage)
N_w = 1      # Wheel gear ratio
N_r1_r2_w = N_r1 * N_r2 * N_w
N_s_r2_w = N_s * N_r2 * N_w
TWO_PI = 2 * math.pi

class HexVehicleController:
    def __init__(self, ws_url="ws://127.0.0.1:8439", control_hz=100, control_mode="speed"):
        self.vehicle_interface  = VehicleAPI(ws_url=ws_url, control_hz=control_hz, control_mode=control_mode)
        self.vehicle = self.vehicle_interface .vehicle

    def get_state(self):
        return {
            "motor_velocity": self.vehicle.get_motor_velocity(),
            "motor_torque": self.vehicle.get_motor_torque(),
            "motor_position": self.vehicle.get_motor_position(),
            "motor_error": self.vehicle.get_motor_error(),
            "vehicle_speed": self.vehicle.get_vehicle_speed(),
        }   

    def set_motor_velocity(self, velocity: list):
        self.vehicle.set_motor_velocity(velocity) # rad/s

    def set_target_vehicle_speed(self, x, y, yaw):
        self.vehicle.set_target_vehicle_speed(x, y, yaw) # m/s, m/s, rad/s

    def set_neutral(self):
        self.vehicle.set_motor_velocity([0.0] * NUM_CASTERS * 2) # rad/s

class CommandType(Enum):
    POSITION = 'position'
    VELOCITY = 'velocity'

# Currently only used for velocity commands
class FrameType(Enum):
    GLOBAL = 'global'
    LOCAL = 'local'

class Vehicle:
    def __init__(
        self,
        max_vel: Tuple[float, float, float] = (0.5, 0.5, 1.57),
        max_accel: Tuple[float, float, float] = (0.25, 0.25, 0.79), 
        ws_url: str = "ws://127.0.0.1:8439",
        control_hz: int = 100,
        control_mode: str = "speed"
    ):
        self.max_vel = np.array(max_vel)
        self.max_accel = np.array(max_accel)
        
        # Use PID file to enforce single instance
        create_pid_file("base-controller")
        
        self.hex_controller = HexVehicleController(ws_url=ws_url, control_hz=control_hz, control_mode=control_mode)
        self.vehicle = self.hex_controller.vehicle
        
        # Joint space
        num_motors = 2 * NUM_CASTERS
        self.q = np.zeros(num_motors)
        self.dq = np.zeros(num_motors)

        # Operational space (global frame)
        self.num_dofs = 3  # (x, y, theta)
        self.x = np.zeros(self.num_dofs)
        self.dx = np.zeros(self.num_dofs)

        # C matrix relating operational space velocities to joint velocities
        self.C = np.zeros((num_motors, self.num_dofs))
        self.C_steer = self.C[::2]
        self.C_drive = self.C[1::2]

        # C_p matrix relating operational space velocities to wheel velocities at the contact points
        self.C_p = np.zeros((num_motors, self.num_dofs))
        self.C_p_steer = self.C_p[::2]
        self.C_p_drive = self.C_p[1::2]
        self.C_p_steer[:, :2] = [1.0, 0.0]
        self.C_p_drive[:, :2] = [0.0, 1.0]

        # C_qp^# matrix relating joint velocities to operational space velocities
        self.C_pinv = np.zeros((num_motors, self.num_dofs))
        self.CpT_Cqinv = np.zeros((self.num_dofs, num_motors))
        self.CpT_Cqinv_steer = self.CpT_Cqinv[:, ::2]
        self.CpT_Cqinv_drive = self.CpT_Cqinv[:, 1::2]

        # OTG (online trajectory generation)
        # Note: It would be better to couple x and y using polar coordinates
        self.otg = Ruckig(self.num_dofs, CONTROL_PERIOD)
        self.otg_inp = InputParameter(self.num_dofs)
        self.otg_out = OutputParameter(self.num_dofs)
        self.otg_res = Result.Working
        self.otg_inp.max_velocity = self.max_vel
        self.otg_inp.max_acceleration = self.max_accel

        # Control loop
        self.command_queue = queue.Queue(1)
        self.control_loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_loop_running = False

    def update_state(self) -> None:
        # Joint positions and velocities
        state = self.hex_controller.get_state()
        
        # forward mapping
        raw_positions = self.motor_mapping(state["motor_position"], "forward", DIRECTION_ARRAY)
        raw_velocities = self.motor_mapping(state["motor_velocity"], "forward", DIRECTION_ARRAY)
        
        self.q = np.array(raw_positions)
        self.dq = np.array(raw_velocities)
        
        q_steer = self.q[::2]

        s = np.sin(q_steer)
        c = np.cos(q_steer)

        # C matrix
        self.C_steer[:, 0] = s / b_x
        self.C_steer[:, 1] = -c / b_x
        self.C_steer[:, 2] = (-h_x*c - h_y*s) / b_x - 1.0
        self.C_drive[:, 0] = c/r - b_y*s / (b_x*r)
        self.C_drive[:, 1] = s/r + b_y*c / (b_x*r)
        self.C_drive[:, 2] = (h_x*s - h_y*c) / r + b_y * (h_x*c + h_y*s) / (b_x*r)

        # C_p matrix
        self.C_p_steer[:, 2] = -b_x*s - b_y*c - h_y
        self.C_p_drive[:, 2] = b_x*c - b_y*s + h_x

        # C_qp^# matrix
        self.CpT_Cqinv_steer[0] = b_x*s + b_y*c
        self.CpT_Cqinv_steer[1] = -b_x*c + b_y*s
        self.CpT_Cqinv_steer[2] = b_x * (-h_x*c - h_y*s - b_x) + b_y * (h_x*s - h_y*c - b_y)
        self.CpT_Cqinv_drive[0] = r * c
        self.CpT_Cqinv_drive[1] = r * s
        self.CpT_Cqinv_drive[2] = r * (h_x*s - h_y*c - b_y)
        with threadpool_limits(limits=1, user_api="blas"):  # Prevent excessive CPU usage
            self.C_pinv = np.linalg.solve(self.C_p.T @ self.C_p, self.CpT_Cqinv)

        # Odometry
        dx_local = self.C_pinv @ self.dq
        theta_avg = self.x[2] + 0.5 * dx_local[2] * CONTROL_PERIOD
        R = np.array(
            [
                [math.cos(theta_avg), -math.sin(theta_avg), 0.0],
                [math.sin(theta_avg), math.cos(theta_avg), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        self.dx = R @ dx_local
        self.x += self.dx * CONTROL_PERIOD

    def start_control(self) -> None:
        if self.control_loop_thread is None:
            print("To initiate a new control loop, please create a new instance of Vehicle.")
            return
        self.control_loop_running = True
        self.control_loop_thread.start()

    def stop_control(self) -> None:
        self.control_loop_running = False
        self.control_loop_thread.join()
        self.control_loop_thread = None

    def control_loop(self) -> None:
        # Set real-time scheduling policy
        try:
            os.sched_setscheduler(
                0,
                os.SCHED_FIFO,
                os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)),
            )
        except PermissionError:
            print("Failed to set real-time scheduling policy, please edit /etc/security/limits.d/99-realtime.conf")

        disable_motors = True
        last_command_time = time.time()
        last_step_time = time.time()

        while self.control_loop_running:
            # Maintain the desired control frequency
            while time.time() - last_step_time < CONTROL_PERIOD:
                time.sleep(0.0001)
            curr_time = time.time()
            step_time = curr_time - last_step_time
            last_step_time = curr_time
            if step_time > 0.01:  # 10 ms
                logging.warning(f"Step time {1000 * step_time:.3f} ms in {self.__class__.__name__} control_loop")

            # Update state
            self.update_state()
            # Global to local frame conversion
            theta = self.x[2]
            R = np.array(
                [
                    [math.cos(theta), math.sin(theta), 0.0],
                    [-math.sin(theta), math.cos(theta), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )

            # Check for new command
            if not self.command_queue.empty():
                command = self.command_queue.get()
                last_command_time = time.time()
                target = command["target"]

                # Velocity command
                if command["type"] == CommandType.VELOCITY:
                    if command["frame"] == FrameType.LOCAL:
                        target = R.T @ target
                    self.otg_inp.control_interface = ControlInterface.Velocity
                    self.otg_inp.target_velocity = np.clip(target, -self.max_vel, self.max_vel)

                # Position command
                elif command["type"] == CommandType.POSITION:
                    self.otg_inp.control_interface = ControlInterface.Position
                    self.otg_inp.target_position = target
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

                self.otg_res = Result.Working
                disable_motors = False
                
            # Maintain current pose if command stream is disrupted
            if time.time() - last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
                self.otg_inp.target_position = self.otg_out.new_position
                self.otg_inp.target_velocity = np.zeros_like(self.dx)
                self.otg_inp.current_velocity = self.dx  # Set this to prevent lurch when command stream resumes
                self.otg_res = Result.Working
                disable_motors = True

            # Slow down base during caster flip
            # Note: At low speeds, this section can be disabled for smoother movement
            if np.max(np.abs(self.dq[::2])) > 12.56:  # Steer joint speed > 720 deg/s
                if self.otg_inp.control_interface == ControlInterface.Position:
                    self.otg_inp.target_position = self.otg_out.new_position
                elif self.otg_inp.control_interface == ControlInterface.Velocity:
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

            # Update OTG
            if self.otg_res == Result.Working:
                self.otg_inp.current_position = self.x
                self.otg_res = self.otg.update(self.otg_inp, self.otg_out)
                self.otg_out.pass_to_input(self.otg_inp)

            if disable_motors:
                # Send motor neutral commands
                self.hex_controller.set_neutral()
                
            else:
                # Operational space velocity
                dx_d = self.otg_out.new_velocity
                dx_d_local = R @ dx_d
                
                # Joint velocities
                dq_d = self.C @ dx_d_local
                
                # inverse mapping
                dq_motor = self.motor_mapping(
                    data=dq_d,
                    mapping_type='inverse',
                    direction_array=DIRECTION_ARRAY
                )

                self.hex_controller.set_motor_velocity(dq_motor.tolist())

    def _enqueue_command(self, command_type: CommandType, target: Any, frame: Optional[FrameType] = None) -> None:
        if self.command_queue.full():
            print("Warning: Command queue is full. Is control loop running?")
        else:
            command = {"type": command_type, "target": target}
            if frame is not None:
                command["frame"] = FrameType(frame)
            self.command_queue.put(command, block=False)

    def set_target_velocity(self, velocity: Any, frame: str = "local") -> None:
        self._enqueue_command(CommandType.VELOCITY, velocity, frame)

    def set_target_position(self, position: Any) -> None:
        self._enqueue_command(CommandType.POSITION, position)


    def motor_mapping(self, data, mapping_type, direction_array=None):
        mapping_array = MOTOR_MAPPING
        if direction_array is None:
            direction_array = np.ones(len(data))
        
        if mapping_type == 'forward':
            mapped_data = [data[i]*direction_array[i] for i in mapping_array]
        elif mapping_type == 'inverse':
            mapped_data = np.zeros_like(data)
            for i, target_idx in enumerate(mapping_array):
                mapped_data[target_idx] = direction_array[target_idx]*data[i]
        else:
            raise ValueError("mapping_type must be 'forward' or 'inverse'")
    
        return mapped_data
    

if __name__ == "__main__":
    
    try:
        vehicle = Vehicle(
            max_vel=(0.5, 0.5, 1.57),
            max_accel=(0.25, 0.25, 0.79),
            ws_url="ws://172.18.2.66:8439",
            control_hz=100,
            control_mode="speed",
        )
        vehicle.start_control()
        for _ in range(50):
            vehicle.set_target_velocity(np.array([0.0, 0.0, 0.39]), frame="global")
            print(f'Vehicle - x: {vehicle.x} dx: {vehicle.dx}')
            time.sleep(POLICY_CONTROL_PERIOD) 

    finally:
        vehicle.stop_control()


