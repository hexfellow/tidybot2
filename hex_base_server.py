#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-30
################################################################

import time
from multiprocessing.managers import BaseManager as MPBaseManager
from hex_base_controller import Vehicle
from constants import BASE_RPC_HOST, BASE_RPC_PORT, RPC_AUTHKEY

class Base:
    def __init__(self, max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.vehicle = None

    def reset(self):
        # Stop low-level control
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()

        # Create new instance of vehicle
        self.vehicle = Vehicle(max_vel=self.max_vel, max_accel=self.max_accel)

        # Start low-level control
        self.vehicle.start_control()
        while not self.vehicle.control_loop_running:
            time.sleep(0.01)

    def execute_action(self, action):
        self.vehicle.set_target_position(action['base_pose'])

    def get_state(self):
        state = {'base_pose': self.vehicle.x}
        return state

    def close(self):
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()

class BaseManager(MPBaseManager):
    pass

BaseManager.register('Base', Base)

if __name__ == '__main__':
    manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f'Base manager server started at {BASE_RPC_HOST}:{BASE_RPC_PORT}')
    server.serve_forever()
    # import numpy as np
    # from constants import POLICY_CONTROL_PERIOD
    # manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    # manager.connect()
    # base = manager.Base()
    # try:
    #     base.reset()
    #     for i in range(50):
    #         base.execute_action({'base_pose': np.array([(i / 50) * 0.5, 0.0, 0.0])})
    #         print(base.get_state())
    #         time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    # finally:
    #     base.close()
