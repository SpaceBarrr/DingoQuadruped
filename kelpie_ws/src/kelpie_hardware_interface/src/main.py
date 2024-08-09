#!/usr/bin/env python
import os
import numpy as np
import rospy
from kelpie.msg import leg_state, joint_states
from kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration


CONFIG = Configuration()
LINKAGE = Leg_linkage(CONFIG)
SERVO_CONTROLLER = ServoInterface(LINKAGE)
def convert_to_array(data):
    joint_angles = np.zeros((3, 4))
    joint_angles[:, 0] = data.fr.roll, data.fr.upper, data.fr.lower
    joint_angles[:, 1] = data.fl.roll, data.fl.upper, data.fl.lower
    joint_angles[:, 2] = data.rr.roll, data.rr.upper, data.rr.lower
    joint_angles[:, 3] = data.rl.roll, data.rl.upper, data.rl.lower
    return joint_angles

def callback(data):
    joint_angles = convert_to_array(data)
    SERVO_CONTROLLER.set_actuator_postions(joint_angles)


print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.init_node('joint_listener', anonymous=True)
rospy.Subscriber("/leg_control/joint_states", joint_states, callback)
rospy.spin()

