from enum import Enum

import numpy as np


class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.20
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0
        self.pid_control = False
        self.joystick_control_active = 1
        self.behavior_state = BehaviorState.REST
        self.euler_orientation = [0, 0, 0]
        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.rotated_foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        self.speed_factor = 1

        self.currently_estopped = 0


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
