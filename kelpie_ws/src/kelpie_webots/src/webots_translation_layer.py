#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""
import sys
import rospy
import ctypes
import os
from math import pi
import numpy as np
from controller import Robot, Gyro, Motor, Accelerometer, InertialUnit
from controller.wb import wb
from kelpie.msg import leg_state, joint_states, imu, att, vec_3d_float32
from numpy import deg2rad
import time



args = rospy.myargv(argv=sys.argv)
DIGITAL_TWIN = int(args[1])
# Set global parameters
SAMPLE_RATE = 1
START_POS = leg_state()
START_POS.roll = 0
START_POS.upper = pi / 8
START_POS.lower = pi / 3

VOLTAGE_MSG = vec_3d_float32()
BATT_VOLTAGE_PUB = rospy.Publisher("/kelpie/battery/voltage", vec_3d_float32, queue_size=1)
IMU_PUBLISHER = rospy.Publisher("/kelpie/imu", imu, queue_size=1)

# Get robot motors
KELPIE = Robot()
T_STEP = int(KELPIE.getBasicTimeStep())

LEG_FL = (
    KELPIE.getDevice('motor.FL_R'),
    KELPIE.getDevice('motor.FL_U'),
    KELPIE.getDevice('motor.FL_L'),
    (1, 1, -1)
)

LEG_FR = (
    KELPIE.getDevice('motor.FR_R'),
    KELPIE.getDevice('motor.FR_U'),
    KELPIE.getDevice('motor.FR_L'),
    (1, -1, 1)
)

LEG_RL = (
    KELPIE.getDevice('motor.RL_R'),
    KELPIE.getDevice('motor.RL_U'),
    KELPIE.getDevice('motor.RL_L'),
    (1, 1, -1)
)

LEG_RR = (
    KELPIE.getDevice('motor.RR_R'),
    KELPIE.getDevice('motor.RR_U'),
    KELPIE.getDevice('motor.RR_L'),
    (1, -1, 1)
)

# Get IMU devices
ACC: Accelerometer = KELPIE.getDevice('IMU.acc')
GYRO: Gyro = KELPIE.getDevice('IMU.gyr')
ATT: InertialUnit = KELPIE.getDevice('IMU.att')

# Init IMU
GYRO.enable(SAMPLE_RATE)
ACC.enable(SAMPLE_RATE)
ATT.enable(SAMPLE_RATE)

# Create messages
IMU_MSG = imu()
IMU_MSG.att = att()
IMU_MSG.acc = vec_3d_float32()
IMU_MSG.gyro = vec_3d_float32()
VOLTAGE_MSG = vec_3d_float32()

# Create joint states global var
JOINT_DATA: joint_states = None
GLOBAL_TIME = time.time()

# Every second, add error to FL and RL upper legs.
ERROR_FL = lambda: (0, (GLOBAL_TIME - time.time()) * deg2rad(0.1), 0)
ERROR_RL = lambda: (0, 0, 0)

ERROR_FR = lambda: (0, (GLOBAL_TIME - time.time()) * deg2rad(0.1), 0)
ERROR_RR = lambda: (0, 0, 0)


ERROR_FL = lambda: (0, 0, 0)
ERROR_RL = lambda: (0, 0, 0)

ERROR_FR = lambda: (0, 0, 0)
ERROR_RR = lambda: (0, 0, 0)

BATT_VALS = lambda: (4.2, 4.2, 8.4)


def set_pos(leg, data: leg_state, error: tuple) -> None:
    """
    Sets the leg position of the robot in Webots
    :param error:
    :param leg: Leg array, containing Roll, Upper, Lower and direction.
    :param data: The leg_state message containing Roll, Upper, Lower angles.
    :return:
    """
    leg[0].setPosition(data.roll * leg[3][0] + error[0])
    leg[1].setPosition(data.upper * leg[3][1] + error[1])
    leg[2].setPosition((data.lower - pi / 2) * leg[3][2] + error[2])


def set_vel(leg, torque: float) -> None:
    """
    Sets max torque. This is unused (and untested).
    :param leg: Leg array, containing Roll, Upper, Lower and direction.
    :param torque: Torque value in Nm
    :return:
    """
    wb.wb_motor_set_velocity(leg[0]._tag, ctypes.c_double(torque))
    wb.wb_motor_set_velocity(leg[1]._tag, ctypes.c_double(torque))
    wb.wb_motor_set_velocity(leg[2]._tag, ctypes.c_double(torque))
    print(leg[0].getMaxVelocity())


def callback(data: joint_states) -> None:
    """
    Callback function for subscriber
    :param data: joint_states message.
    :return: None
    """
    global JOINT_DATA
    JOINT_DATA = data


def init_pos():
    """
    Initialise sim to a certain start position.
    :return:
    """
    set_pos(LEG_FL, START_POS)
    set_pos(LEG_FR, START_POS)
    set_pos(LEG_RL, START_POS)
    set_pos(LEG_RR, START_POS)


#init_pos()


# Initialise ROS nodes
print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.init_node('webots_translation_layer', anonymous=True)
rospy.Subscriber("/kelpie/leg_control/joint_states", joint_states, callback, queue_size=1)

print('Running the control loop')
while KELPIE.step(T_STEP) != -1 and not rospy.is_shutdown():
    acc = ACC.getValues()
    gyro = GYRO.getValues()
    # TODO: Change to quaternion
    att = ATT.getRollPitchYaw()

    IMU_MSG.att.roll, IMU_MSG.att.pitch, IMU_MSG.att.yaw = att[1], -(att[0] - pi / 2), att[2]
    # print(IMU_MSG.att)
    IMU_MSG.acc.x, IMU_MSG.acc.y, IMU_MSG.acc.z = acc[0], acc[1], acc[2]
    IMU_MSG.gyro.x, IMU_MSG.gyro.y, IMU_MSG.gyro.z = gyro[0], gyro[1], gyro[2]
    if not DIGITAL_TWIN:
        IMU_PUBLISHER.publish(IMU_MSG)
        VOLTAGE_MSG.x = BATT_VALS[0]        # Cell 1 voltage
        VOLTAGE_MSG.y = BATT_VALS[1]        # Cell 2 voltage
        VOLTAGE_MSG.z = BATT_VALS[2]        # Raw battery voltage
        BATT_VOLTAGE_PUB.publish(VOLTAGE_MSG)

    if JOINT_DATA is None:
        # Skip if no joint data has been received.
        continue

    set_pos(LEG_FL, JOINT_DATA.fl, ERROR_FL())
    set_pos(LEG_FR, JOINT_DATA.fr, ERROR_FR())
    set_pos(LEG_RL, JOINT_DATA.rl, ERROR_RL())
    set_pos(LEG_RR, JOINT_DATA.rr, ERROR_RR())
