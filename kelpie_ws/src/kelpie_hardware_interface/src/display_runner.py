#!/usr/bin/env python
import os
import rospy
import time

from kelpie_hardware_interface.batt_sense.publisher import publish as publish_batt_v
from kelpie_hardware_interface.lcd.kelpie_display import KelpieDisplay
from kelpie_common.Utilities import RollingAverage
from kelpie.msg import vec_3d_float32

BATT_VOLTAGE_PUB = rospy.Publisher("/kelpie/battery/voltage", vec_3d_float32, queue_size=1)

if __name__ == '__main__':
    print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
    rospy.init_node('kelpie_display', anonymous=True)

    # Start main subscribers
    batt_display = KelpieDisplay()
    batt_avg = RollingAverage(window=40, initial=0)

    # Handle publishers
    while True:
        batt_avg.append(round(publish_batt_v(BATT_VOLTAGE_PUB), 2))
        batt_display.battery_v = batt_avg.average
        time.sleep(0.05)
        pass

