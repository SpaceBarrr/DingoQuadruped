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

import os
import time
from math import pi
import rospy
from controller import Robot
from std_msgs.msg import Float64
from kelpie.msg import leg_state, joint_states

KELPIE = Robot()
LEG_FL = (
    KELPIE.getDevice('motor.FL_R'),
    KELPIE.getDevice('motor.FL_U'),
    KELPIE.getDevice('motor.FL_L'),
    (1, -1, -1)
)

LEG_FR = (
    KELPIE.getDevice('motor.FR_R'),
    KELPIE.getDevice('motor.FR_U'),
    KELPIE.getDevice('motor.FR_L'),
    (-1, 1, 1)
)

LEG_RL = (
    KELPIE.getDevice('motor.RL_R'),
    KELPIE.getDevice('motor.RL_U'),
    KELPIE.getDevice('motor.RL_L'),
    (-1, -1, -1)
)

LEG_RR = (
    KELPIE.getDevice('motor.RR_R'),
    KELPIE.getDevice('motor.RR_U'),
    KELPIE.getDevice('motor.RR_L'),
    (1, 1, 1)
)


T_STEP = int(KELPIE.getBasicTimeStep())

START_POS = leg_state()
START_POS.roll = 0
START_POS.upper = pi / 8
START_POS.lower = pi/3

def set_pos(leg, data):
    leg[0].setPosition(data.roll * leg[3][0])
    leg[1].setPosition(data.upper * leg[3][1])
    leg[2].setPosition(data.lower * leg[3][2])


def callback(data):
    set_pos(LEG_FL, data.fl)
    set_pos(LEG_FR, data.fr)
    set_pos(LEG_RL, data.rl)
    set_pos(LEG_RR, data.rr)

def init():
    set_pos(LEG_FL, START_POS)
    set_pos(LEG_FR, START_POS)
    set_pos(LEG_RL, START_POS)
    set_pos(LEG_RR, START_POS)

init()
print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.init_node('webot_joint_listener', anonymous=True)
rospy.Subscriber("/leg_control/joint_states", joint_states, callback)

print('Running the control loop')

while KELPIE.step(T_STEP) != -1 and not rospy.is_shutdown():
    pass

