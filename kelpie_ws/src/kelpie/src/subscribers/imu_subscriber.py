#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import os
import rospy
from kelpie.msg import imu
import numpy as np
class ImuSubscriber:
    def __init__(self) -> None:
        """
        Initialise the ImuSubscriber class
        """
        # roll pitch yaw
        self.att = np.zeros((3,))

        # xyz
        self.acc = np.zeros((3,))
        self.gyro = np.zeros((3,))
        rospy.Subscriber("/kelpie/imu", imu, self.callback, queue_size=1)

    def callback(self, imu_data: imu) -> None:
        """
        Callback function for IMU subscriber.
        :param imu_data: The IMU data, expected as imu message format
        :return: None
        """
        # Keep pointer/reference. Do not overwrite with new class reference.
        self.att[:] = imu_data.att.roll, imu_data.att.pitch, imu_data.att.yaw
        self.acc[:] = imu_data.acc.x, imu_data.acc.y, imu_data.acc.z
        self.gyro[:] = imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z

        self.att = np.round(self.att, decimals=3)
        self.acc = np.round(self.acc, decimals=3)
        self.gyro = np.round(self.gyro, decimals=3)

    @property
    def roll(self):
        return self.att[0]

    @property
    def pitch(self):
        return self.att[1]

    @property
    def yaw(self):
        return self.att[2]



