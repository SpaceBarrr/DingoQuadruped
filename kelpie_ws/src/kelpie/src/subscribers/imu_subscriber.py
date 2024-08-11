#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import os
import rospy
from kelpie.msg import imu, att, xyz_float32
import numpy as np
class ImuSubscriber:
    def __init__(self):
        """
        Initialise the ImuSubscriber class
        """
        self.att = np.zeros((3,))
        self.acc = np.zeros((3,))
        self.gyro = np.zeros((3,))
        rospy.Subscriber("/kelpie/imu", imu, self.callback, queue_size=1)

    def callback(self, imu_data: imu):
        """
        Callback function for IMU subscriber.
        :param imu_data: The IMU data, expected as imu message format
        :return: None
        """
        self.att[:] = imu_data.att.roll, imu_data.att.pitch, imu_data.att.yaw
        self.acc[:] = imu_data.acc.x, imu_data.acc.y, imu_data.acc.z
        self.gyro[:] = imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z

        self.att = np.round(self.att, 4)
        self.acc = np.round(self.acc, 4)
        self.gyro = np.round(self.gyro, 4)

        print(self.att)
        print(self.acc)
        print(self.gyro)
        print("")


