#!/usr/bin/env python
import os
import numpy as np
import rospy
from kelpie_hardware_interface.servo.servo_subscriber import ServoSubscriber



if __name__ == '__main__':
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    rospy.init_node('kelpie_hardware', anonymous=True)

    # Start subscribers
    servo_subscriber = ServoSubscriber()
    while True:
        # Publish data
        pass


    rospy.spin()

