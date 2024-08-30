#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import os
import rospy
from kelpie.msg import joint_states
import numpy as np
class MotorCurrentSubscriber:
    def __init__(self) -> None:
        """
        Initialise the ImuSubscriber class
        """
        self.fr = np.zeros((3,))
        self.fl = np.zeros((3,))
        self.rr = np.zeros((3,))
        self.rl = np.zeros((3,))
        rospy.Subscriber("/kelpie/leg_control/currents", joint_states, self.callback, queue_size=1)

    def callback(self, current_data: joint_states) -> None:
        """
        Callback function for IMU subscriber.
        :param current_data: The IMU data, expected as imu message format
        :return: None
        """
        # Keep pointer/reference. Do not overwrite with new class reference.
        self.fr[:] = current_data.fr.roll, current_data.fr.upper, current_data.fr.lower
        self.fl[:] = current_data.fr.roll, current_data.fr.upper, current_data.fr.lower
        self.rr[:] = current_data.fr.roll, current_data.fr.upper, current_data.fr.lower
        self.rr[:] = current_data.fr.roll, current_data.fr.upper, current_data.fr.lower



