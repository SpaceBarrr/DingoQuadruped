#!/usr/bin/env python

import rospy
from kelpie.msg import vec_3d_float32
import numpy as np

class BatteryVoltageSubscriber:
    def __init__(self) -> None:
        """
        Initialise the BatteryVoltageSubscriber class
        """
        self.voltages = np.zeros((3,))
        
        rospy.Subscriber('/kelpie/battery/voltage', vec_3d_float32, self.callback, queue_size=1)

    def callback(self, batt_data: vec_3d_float32) -> None:
        """
        Callback function for batt voltage subscriber.
        :param batt_data: The batt voltage data msg
        :return: None
        """
        # Keep pointer/reference. Do not overwrite with new class reference.
        self.voltages[0], self.voltages[1], self.voltages[2] = batt_data.x, batt_data.y, batt_data.z
        
    def get_voltages(self) -> list:
        '''
        Returns a list of all voltage data
        :param: None
        :return: a list of cell voltage of the format [cell1, cell2, overall_voltage]
        '''
        return self.voltages


