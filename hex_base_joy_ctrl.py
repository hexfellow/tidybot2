#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-30
################################################################

import time
import pygame
import numpy as np
from hex_base_controller import Vehicle

if __name__ == "__main__":
    vehicle = Vehicle(
        max_vel=(0.5, 0.5, 1.0),
        max_accel=(1.0, 1.0, 1.0),
    )
    vehicle.start_control()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick/gamepad connected!")
        exit()
    else:
        print(f"Detected {pygame.joystick.get_count()} joystick(s).")

    # Initialize the joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")

    try:
        # Main loop to read joystick inputs
        # cmd_vel: [vel_x, vel_y, omega]
        cmd_vel = np.array([0.0, 0.0, 0.0])
        while True:
            # get handle input
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    axis_0 = joystick.get_axis(0)
                    axis_1 = -joystick.get_axis(1)
                    axis_2 = joystick.get_axis(2)
                    axis_3 = -joystick.get_axis(3)

                    cmd_vel[0] = axis_1 * 0.5
                    cmd_vel[1] = -axis_0 * 0.5
                    cmd_vel[2] = axis_3 * 1.0

            cmd_vel[np.abs(cmd_vel) < 0.03] = 0.0
            print(
                f"cmd_vel: ({cmd_vel[0]:.2f}, {cmd_vel[1]:.2f}, {cmd_vel[2]:.2f})"
            )
            vehicle.set_target_velocity(cmd_vel, frame="local")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
        vehicle.stop_control()
