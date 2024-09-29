#!/usr/bin/env python
import os
import rospy
import time

from kelpie_hardware_interface.servo.servo_subscriber import ServoSubscriber
from kelpie_hardware_interface.handheld_controller.ps4 import ControllerInterface
from kelpie_hardware_interface.current_sense.publisher import publish as publish_currents
from kelpie_hardware_interface.imu.publisher import publish as publish_imu
from kelpie_hardware_interface.batt_sense.publisher import publish as publish_batt_v
from kelpie_hardware_interface.lcd.kelpie_display import KelpieDisplay
from kelpie_common.Utilities import RollingAverage
from kelpie.msg import joint_states, vec_3d_float32, imu

MOTOR_CURRENT_PUB = rospy.Publisher("/kelpie/leg_control/currents", joint_states, queue_size=1)
BATT_VOLTAGE_PUB = rospy.Publisher("/kelpie/battery/voltage", vec_3d_float32, queue_size=1)
IMU_PUBLISHER = rospy.Publisher("/kelpie/imu", imu, queue_size=1)


if __name__ == '__main__':
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    rospy.init_node('kelpie_hardware', anonymous=True)

    # Start main subscribers
    servo_subscriber = ServoSubscriber()
    controller = ControllerInterface()
    batt_display = KelpieDisplay()
    batt_avg = RollingAverage(window=40, initial=0)

    # Handle publishers
    while True:
        publish_currents(MOTOR_CURRENT_PUB)
        batt_avg.append(round(publish_batt_v(BATT_VOLTAGE_PUB), 2))
        batt_display.battery_v = batt_avg.average
        publish_imu(IMU_PUBLISHER)
        time.sleep(0.05)
        pass

