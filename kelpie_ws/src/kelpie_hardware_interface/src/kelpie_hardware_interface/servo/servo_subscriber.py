#!/usr/bin/env python
import os
import numpy as np
import rospy
from kelpie.msg import leg_state, joint_states
from kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration

class ServoSubscriber:
    def __init__(self):
        self.servo_interface = ServoInterface(Leg_linkage(Configuration()))
        rospy.Subscriber("/kelpie/leg_control/joint_states", joint_states, self.callback)
        print("started")

    def callback(self, data):
        joint_angles = self.convert_to_array(data)
        self.servo_interface.set_actuator_postions(joint_angles)

    @staticmethod
    def convert_to_array(data):
        joint_angles = np.zeros((3, 4))
        joint_angles[:, 0] = data.fr.roll, data.fr.upper, data.fr.lower
        joint_angles[:, 1] = data.fl.roll, data.fl.upper, data.fl.lower
        joint_angles[:, 2] = data.rr.roll, data.rr.upper, data.rr.lower
        joint_angles[:, 3] = data.rl.roll, data.rl.upper, data.rl.lower
        return joint_angles




