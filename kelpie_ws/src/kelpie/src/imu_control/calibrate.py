#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import numpy as np
import rospy
from subscribers.imu_subscriber import ImuSubscriber
from subscribers.motor_current_subscriber import MotorCurrentSubscriber

class Calibrator:
    def __init__(self, imu: ImuSubscriber, motor_currents: MotorCurrentSubscriber, joint_publisher: rospy.Publisher):
        """
        Initialiser for calibrator class.
        :param imu: Imu subscriber class
        :param motor_current: Roll motor currents (FL, FR, RL, RR). This should be passed as a reference, not a copy!
        """
        self.imu = imu
        self.motor_currents = motor_currents
        self.publisher = joint_publisher

    def run(self):
        self.zero_roll()
        self.zero_upper()
        self.zero_lower()

    def zero_roll(self):
        pass

    def zero_upper(self):
        pass

    def zero_lower(self):
        pass

