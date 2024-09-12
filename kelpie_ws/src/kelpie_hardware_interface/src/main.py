#!/usr/bin/env python
import os
import rospy
import time

from kelpie_hardware_interface.servo.servo_subscriber import ServoSubscriber
from kelpie_hardware_interface.handheld_controller.ps4 import Ps4Interface
from kelpie_hardware_interface.current_sense.publisher import publish as publish_currents
from kelpie_hardware_interface.imu.publisher import publish as publish_imu
from kelpie_hardware_interface.batt_sense.publisher import publish as publish_batt_v
from kelpie.msg import joint_states, xyz_float32, imu

MOTOR_CURRENT_PUB = rospy.Publisher("/kelpie/leg_control/currents", joint_states, queue_size=1)
BATT_VOLTAGE_PUB = rospy.Publisher("/kelpie/battery/voltage", xyz_float32, queue_size=1)
IMU_PUBLISHER = rospy.Publisher("/kelpie/imu", imu, queue_size=1)

if __name__ == '__main__':
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    rospy.init_node('kelpie_hardware', anonymous=True)

    # Start main subscribers
    servo_subscriber = ServoSubscriber()
    ps4 = Ps4Interface()

    # Handle publishers
    while True:
        publish_currents(MOTOR_CURRENT_PUB)
        publish_batt_v(BATT_VOLTAGE_PUB)
        publish_imu(IMU_PUBLISHER)
        time.sleep(0.01)
        pass
