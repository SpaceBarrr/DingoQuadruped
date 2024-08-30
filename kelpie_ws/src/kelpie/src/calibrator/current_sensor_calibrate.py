#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import numpy as np
import rospy
from subscribers.imu_subscriber import ImuSubscriber
from subscribers.motor_current_subscriber import MotorCurrentSubscriber
from kelpie_common.Config import ServoIndex as s_idx
import time
from kelpie.msg import joint_states
from kelpie.msg import leg_state
from kelpie_common.Utilities import build_leg_msg, RollingAverage
from kelpie_hardware_interface.current_sense.current_sensor import SensorIdx, MotorChan
class Calibrator:
    def __init__(self, imu: ImuSubscriber, motor_currents: MotorCurrentSubscriber, joint_publisher: rospy.Publisher):
        """
        Initialiser for calibrator class.
        :param imu: Imu subscriber class
        :param motor_currents: Roll motor currents (FL, FR, RL, RR). This should be passed as a reference, not a copy!
        """
        self.imu = imu
        self.motor_currents = motor_currents
        self.publisher = joint_publisher

        self.joint_states_msg = joint_states()
        self.fl_state_msg = leg_state()
        self.fr_state_msg = leg_state()
        self.rl_state_msg = leg_state()
        self.rr_state_msg = leg_state()
        self.joint_angles = np.zeros((3, 4))
        self.rolling_avg_curr = RollingAverage(window=3)

    def run(self, init_state):
        """
        Main calibration script.
        :param init_state: The current state of the robot to prevent sudden movement of servo's
        :return: None
        """
        self.joint_angles = init_state
        #self._settle()
        #self._zero_roll()
        self._zero_lower()
        self._zero_upper()


    def _hit_limit(self, servo):
        sensor = SensorIdx[servo].value
        channel = MotorChan[servo].value
        while not self._collided(sensor, channel):
            self.joint_angles[s_idx[servo].value] -= 0.5 * 0.0174
            self.publisher.publish(self.joint_states_msg)

        self.joint_angles[s_idx[servo].value] += 2 * 0.0174
        self.publisher.publish(self.joint_states_msg)
        print("Hit Limit")

    def _zero_roll(self):
        while True:
            pass
        # TODO: Create method for zeroing roll motors
        pass

    def _zero_upper(self):
        # TODO: Create method for zeroing upper motors
        pass

    def _zero_lower(self):
        # TODO: Create method for zeroing lower motors
        pass

    def _collided(self, sensor, channel):
        """
        Calculates the rolling average and determines if the IMU has settled or not.
        :return: Bool
        """
        # Calculate magnitude and append to rolling average
        current = self.motor_currents.get_shunt_current(sensor, channel)
        self.rolling_avg_curr.append(current)
        print(self.rolling_avg_curr.average)

        # Determine if settled. Earths maximum gravity is 9.83, taking 9.85 as threshold.
        return self.rolling_avg_curr.average < 1



