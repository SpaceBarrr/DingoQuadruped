#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import numpy as np
import rospy
from kelpie_common.Config import ServoIndex as s_idx
import time
from kelpie.msg import joint_states
from kelpie.msg import leg_state
from kelpie_common.Utilities import build_leg_msg, RollingAverage
from kelpie_hardware_interface.current_sense.current_sensor import SensorIdx, MotorChan
class Calibrator:
    def __init__(self, imu, motor_currents, joint_publisher: rospy.Publisher):
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
        self.rolling_avg_curr = RollingAverage(window=5, initial=0)

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

    def _publish(self):
        self.joint_states_msg.fr = build_leg_msg(self.fr_state_msg, self.joint_angles[:, 0])
        self.joint_states_msg.fl = build_leg_msg(self.fl_state_msg, self.joint_angles[:, 1])
        self.joint_states_msg.rr = build_leg_msg(self.rr_state_msg, self.joint_angles[:, 2])
        self.joint_states_msg.rl = build_leg_msg(self.rl_state_msg, self.joint_angles[:, 3])
        self.publisher.publish(self.joint_states_msg)

    def _hit_limit(self, servo, step=1, backoff=-2, tstep=0.1, limit=0.1):
        while not self._collided(servo, limit=limit):
            self.joint_angles[s_idx[servo].value] += step * 0.0174
            self._publish()
            time.sleep(tstep)

        self.joint_angles[s_idx[servo].value] += backoff * 0.0174
        self._publish()
        print(self.rolling_avg_curr.average)
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
        self._hit_limit("RR_L", step=0.5, backoff=-10, tstep=0.05, limit=0.05)
        self.rolling_avg_curr.reset()
        time.sleep(0.5)
        self._hit_limit("RR_L", step=0.1, backoff=-0.5, tstep=0.01, limit=0.05)
        pass

    def _collided(self, servo, limit=0.1):
        """
        Calculates the rolling average and determines if the IMU has settled or not.
        :return: Bool
        """
        # Calculate magnitude and append to rolling average
        current = self.motor_currents.get_current(servo)
        self.rolling_avg_curr.append(current)
        #print(self.rolling_avg_curr.average)

        # Determine if settled. Earths maximum gravity is 9.83, taking 9.85 as threshold.
        return self.rolling_avg_curr.average >= limit



