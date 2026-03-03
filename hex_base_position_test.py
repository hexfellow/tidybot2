#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-30
################################################################

import time
import numpy as np
from hex_base_controller import Vehicle

SAVE_TRAJ = True
TAU = 2 * np.pi
FILE_NAME = "odom_tum.txt"


def write_tum(f, ts, state):
    x, y, yaw = state
    half_yaw = 0.5 * ((yaw + np.pi) % TAU - np.pi)
    qw = np.cos(half_yaw)
    qz = np.sin(half_yaw)
    f.write(
        f"{ts:.6f} {x:.6f} {y:.6f} 0.000000 0.000000 0.000000 {qz:.6f} {qw:.6f}\n"
    )


if __name__ == "__main__":
    # Initialize the trajectory file
    if SAVE_TRAJ:
        traj_file = open(FILE_NAME, "w")

    # Initialize the vehicle
    vehicle = Vehicle(
        max_vel=(0.5, 0.5, 1.0),
        max_accel=(1.0, 1.0, 1.0),
    )
    vehicle.start_control()

    # Target position and arrival detection parameters
    target_position = np.array([0.25, 0.15, 0.5])
    position_threshold = 0.005  # 0.5cm for x,y position
    angle_threshold = 0.01  # ~1 degrees for orientation
    velocity_threshold = 0.01  # Almost stopped
    target_reached = False

    try:
        while True:
            # Check if target is reached
            if not target_reached:
                position_error = np.linalg.norm(vehicle.x[:2] -
                                                target_position[:2])
                angle_error = abs(vehicle.x[2] - target_position[2])
                velocity_norm = np.linalg.norm(vehicle.dx)

                if (position_error < position_threshold
                        and angle_error < angle_threshold
                        and velocity_norm < velocity_threshold):
                    target_reached = True
                    # Switch to velocity control mode and send zero velocity to stop
                    vehicle.set_target_position(target_position)
                    print(
                        f"Target reached! Position error: {position_error*1000:.1f}mm, "
                        f"Angle error: {np.degrees(angle_error):.1f}deg")
                else:
                    vehicle.set_target_position(target_position)
                    print(
                        f"Moving to target... Position error: {position_error*1000:.1f}mm, "
                        f"Velocity: {velocity_norm:.3f}m/s")

            if SAVE_TRAJ:
                write_tum(traj_file, time.time(), vehicle.x)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
        vehicle.stop_control()

        if SAVE_TRAJ:
            traj_file.close()
