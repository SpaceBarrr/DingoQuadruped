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
        self.rolling_avg_acc = RollingAverage(window=10, initial=np.inf)
        self.rolling_avg_gyro = RollingAverage(window=10, initial=np.inf)

    def run(self, init_state):
        """
        Main calibration script.
        :param init_state: The current state of the robot to prevent sudden movement of servo's
        :return: None
        """
        self.joint_angles = init_state
        self._settle()
        self._zero_roll()
        self._zero_upper()
        self._zero_lower()

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

    def _settled(self):
        """
        Calculates the rolling average and determines if the IMU has settled or not.
        :return: Bool
        """
        # Calculate magnitude and append to rolling average
        self.rolling_avg_acc.append(np.linalg.norm(self.imu.acc))
        self.rolling_avg_gyro.append(np.linalg.norm(self.imu.gyro))
        print(self.imu.gyro)
        print(self.imu.acc)

        # Determine if settled. Earths maximum gravity is 9.83, taking 9.85 as threshold.
        return self.rolling_avg_gyro.average < 0.01 and self.rolling_avg_acc.average < 9.85

    def _settle(self):
        """
        Starts the settling routine, does this by creating a rolling average of the IMU's acceleration and gyro magnitudes.
        :return: None
        """
        while not self._settled():
            self.joint_angles[s_idx.FL_U.value] -= 0.5 * 0.0174
            self.joint_angles[s_idx.FR_U.value] -= 0.5 * 0.0174
            self.joint_angles[s_idx.RL_U.value] -= 0.5 * 0.0174
            self.joint_angles[s_idx.RR_U.value] -= 0.5 * 0.0174
            self.joint_angles[s_idx.FL_L.value] += 0.5 * 0.0174
            self.joint_angles[s_idx.FR_L.value] += 0.5 * 0.0174
            self.joint_angles[s_idx.RL_L.value] += 0.5 * 0.0174
            self.joint_angles[s_idx.RR_L.value] += 0.5 * 0.0174

            self.joint_states_msg.fr = build_leg_msg(self.fr_state_msg, self.joint_angles[:, 0])
            self.joint_states_msg.fl = build_leg_msg(self.fl_state_msg, self.joint_angles[:, 1])
            self.joint_states_msg.rr = build_leg_msg(self.rr_state_msg, self.joint_angles[:, 2])
            self.joint_states_msg.rl = build_leg_msg(self.rl_state_msg, self.joint_angles[:, 3])

            self.publisher.publish(self.joint_states_msg)

            time.sleep(0.1)
        print("Settled")


