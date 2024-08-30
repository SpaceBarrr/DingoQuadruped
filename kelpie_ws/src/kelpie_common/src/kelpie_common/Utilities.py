import numpy as np
from collections import deque

def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)

def build_leg_msg(msg, angles):
    msg.roll, msg.upper, msg.lower = angles[0], angles[1], angles[2]
    return msg


class RollingAverage:
    def __init__(self, window=10):
        self.window = window
        self._circular_queue = deque([np.inf]*window, maxlen=window)

    def append(self, value):
        self._circular_queue.append(value)

    def reset(self):
        self._circular_queue = deque([np.inf] * self.window, maxlen=self.window)

    @property
    def average(self):
        return np.average(self._circular_queue)

