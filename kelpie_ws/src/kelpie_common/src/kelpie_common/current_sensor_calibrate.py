#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024

import numpy as np
import rospy
from kelpie_common.Config import ServoIndex as s_idx
import time
from kelpie.msg import joint_states
from kelpie.msg import leg_state
from kelpie_common.Utilities import build_leg_msg, RollingAverage, format_angles
from kelpie_hardware_interface.current_sense.current_sensor import SensorIdx, MotorChan
from threading import Thread
class Calibrator:
    def __init__(self, motor_currents, imu=None, joint_publisher: rospy.Publisher=None):
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

        # Below is for calibration when used in the hardware script.
        self.servo_angles = None
        self.servo_interface = None
        self._control_motor = self._publish if joint_publisher is not None else self._hardware_control
        self._motor_control_thread = None
        self._write_sync = [False, False, False, False]
        self._write_delay = 0.05

    def _hardware_control(self):
        self.servo_interface.physical_calibration_offsets = self.joint_angles
        self.servo_interface.set_servo_angles(self.servo_angles)

    def _publish(self):
        self.joint_states_msg.fr = build_leg_msg(self.fr_state_msg, self.joint_angles[:, 0])
        self.joint_states_msg.fl = build_leg_msg(self.fl_state_msg, self.joint_angles[:, 1])
        self.joint_states_msg.rr = build_leg_msg(self.rr_state_msg, self.joint_angles[:, 2])
        self.joint_states_msg.rl = build_leg_msg(self.rl_state_msg, self.joint_angles[:, 3])
        self.publisher.publish(self.joint_states_msg)


    def run(self, init_state):
        """
        Main calibration script.
        :param init_state: The current state of the robot to prevent sudden movement of servo's
        :return: None
        """
        self.joint_angles = init_state

        fr = Thread(target=self._cal_leg, args=("FR",))
        fl = Thread(target=self._cal_leg, args=("FL",))
        rr = Thread(target=self._cal_leg, args=("RR",))
        rl = Thread(target=self._cal_leg, args=("RL",))

        fr.start()
        fl.start()
        rr.start()
        rl.start()

        fr.join()
        fl.join()
        rr.join()
        rl.join()

    def _cal_leg(self, leg):
        """
        Main calibration script.
        :param init_state: The current state of the robot to prevent sudden movement of servo's
        :return: None
        """
        #self._settle()
        #self._zero_roll()
        self._zero_lower(leg)
        self._zero_upper(leg)

    def _zero_roll(self, leg):
        while True:
            pass
        # TODO: Create method for zeroing roll motors
        pass

    def _zero_upper(self, leg):
        # TODO: Create method for zeroing upper motors
        pass

    def _zero_lower(self, leg):
        # TODO: Create method for zeroing lower motors
        self._hit_limit(f"{leg}_L", step=0.5, backoff=-10, tstep=0.05, limit=0.05)
        self.rolling_avg_curr.reset()
        time.sleep(0.5)
        self._hit_limit(f"{leg}_L", step=0.1, backoff=-0.5, tstep=0.01, limit=0.05)
        return

    def _hit_limit(self, servo, step=1., backoff=-2., tstep=0.1, limit=0.1):
        while not self._collided(servo, limit=limit):
            # Run until collided.
            if self._write_sync[s_idx[servo].value[1]]:
                # If its associated bit is True, that means it has already been written too, and is waiting to be cleared by main motor control thread.
                continue

            self.joint_angles[s_idx[servo].value] += step * 0.0174
            self._write_sync[s_idx[servo].value[1]] = True      # Set bit to True
            time.sleep(tstep)

        if self._write_sync[s_idx[servo].value[1]]:
            # If its associated bit is True, that means it has already been written too, and is waiting to be cleared by main motor control thread.
            time.sleep(self._write_delay*1.1)   # Wait slightly longer than the frequency of the write thread.

        self.joint_angles[s_idx[servo].value] += backoff * 0.0174
        self._write_sync[s_idx[servo].value[1]] = True
        print(self.rolling_avg_curr.average)
        print("Hit Limit")

    def _collided(self, servo, limit=0.1):
        """
        Calculates the rolling average and determines if the IMU has settled or not.
        :return: Bool
        """
        # Calculate magnitude and append to rolling average
        current = self.motor_currents.get_current(servo)
        self.rolling_avg_curr.append(current)

        return self.rolling_avg_curr.average >= limit

    def _motor_control_thread(self):
        """
        Main motor control thread. This is to allow multiple threads to control the motors without simultaneous I2C/Publish write requests.
        """
        while sum(self._write_sync) != -4:      # If sum of write sync is -4, that means all bits are -1 and thus completed.
            if self._write_synced():
                self._control_motor()
                self._reset_write_sync()


    def _write_synced(self):
        """
        Checks if all bits in write_sync is set to True. This indicates that it is ready for a write operation. If item is -1, it means it has been completed, ignore.
        """
        for item in self._write_sync:
            if item != -1 and not item:
                return False
        return True

    def _reset_write_sync(self):
        """
        Resets write_synced attribute to False. Indicated it has been written too. If the item is -1, it means it has been completed and hashed out.
        """
        for i, item in enumerate(self._write_sync):
            self._write_sync[i] = True if item == -1 else False

