#!/usr/bin/env python
import csv
import os
import time
import rospy
from kelpie.msg import imu
import numpy as np

FP = __file__.replace("/src/main.py", "/logs/")
FILE_NAME = FP + time.strftime("%Y%m%d-%H%M%S") + ".csv"

att = np.zeros((4,))
acc = np.zeros((4,))
gyro = np.zeros((4,))
REF_TIME = time.time()
rospy.init_node("kelpie_logger")

def IMU_callback(imu_data: imu) -> None:
    """
    Callback function for IMU subscriber.
    :param imu_data: The IMU data, expected as imu message format
    :return: None
    """
    # Keep pointer/reference. Do not overwrite with new class reference.
    att[:] = time.time() - REF_TIME ,imu_data.att.roll, imu_data.att.pitch, imu_data.att.yaw
    acc[:] = time.time() - REF_TIME ,imu_data.acc.x, imu_data.acc.y, imu_data.acc.z
    gyro[:] = time.time() - REF_TIME ,imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z

IMU = rospy.Subscriber("/kelpie/imu", imu, IMU_callback, queue_size=1)

if not os.path.exists(FP):
    os.makedirs(FP)

with open(FILE_NAME,mode="w+", newline='') as csvfile:
    while True:
        spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|')
        spamwriter.writerow(att)
        time.sleep(0.1)