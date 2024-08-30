#!/usr/bin/env python
import os
import numpy as np
import rospy
from kelpie_hardware_interface.servo.servo_subscriber import ServoSubscriber
from kelpie_hardware_interface.handheld_controller.ps4 import Ps4Interface
import time


if __name__ == '__main__':
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    rospy.init_node('kelpie_hardware', anonymous=True)

    # Start main subscribers
    servo_subscriber = ServoSubscriber()
    ps4 = Ps4Interface()
    while True:
        time.sleep(0.1)
        pass

