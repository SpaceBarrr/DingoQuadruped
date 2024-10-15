#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import os
import rospy
from kelpie.msg import joint_states
import numpy as np
from kelpie_common.Config import ServoIndex as s_idx
class MotorCurrentSubscriber:
    def __init__(self) -> None:
        """
        Initialise the MotorCurrentSubscriber class
        """
        self.currents = np.zeros((3, 4))
        
        rospy.Subscriber("/kelpie/leg_control/currents", joint_states, self.callback, queue_size=1)

    def callback(self, current_data: joint_states) -> None:
        """
        Callback function for current subscriber.
        :param current_data: The current data, expected as current message format
        :return: None
        """
        # Keep pointer/reference. Do not overwrite with new class reference.
        self.currents[:, 0] = current_data.fr.roll, current_data.fr.upper, current_data.fr.lower
        self.currents[:, 1] = current_data.fl.roll, current_data.fl.upper, current_data.fl.lower
        self.currents[:, 2] = current_data.rr.roll, current_data.rr.upper, current_data.rr.lower
        self.currents[:, 3] = current_data.rl.roll, current_data.rl.upper, current_data.rl.lower
        
    def get_current(self, servo):
        return self.currents[s_idx[servo].value]


