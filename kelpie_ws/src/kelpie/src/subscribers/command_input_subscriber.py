#!/usr/bin/env python

# Written by: William L
# Last Modified 11/08/2024


import numpy as np
import rospy
from gait_controller.Command import Command
from gait_controller.State import BehaviorState
from kelpie_common.Utilities import deadband, clipped_first_order_filter
#from sensor_msgs.msg import Joy
from kelpie.msg import commands


class InputSubscriber:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0
        self.previous_calibrate_toggle = 0
        self.previous_pid_control_toggle = False

        self.rounding_dp = 2

        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0
        self.calibrate_event = 0
        self.pid_control_event = False

        self.input_messages = rospy.Subscriber("/kelpie/command_input", commands, self.input_callback)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

    def input_callback(self, command):
        gait_toggle = command.gait_toggle
        hop_toggle = command.hop_toggle
        joystick_toggle = command.joystick_toggle
        calibrate_toggle = command.calibration_toggle
        pid_toggle = command.pid_toggle

        self.developing_command = Command()
        # Discrete commands
        # Check if requesting a state transition to trotting, or from trotting to resting

        if not self.trot_event:
            self.trot_event = (gait_toggle and not self.previous_gait_toggle)

        # Check if requesting a state transition to hopping, from trotting or resting
        if not self.hop_event:
            self.hop_event = (hop_toggle and not self.previous_hop_toggle)

        if not self.joystick_control_event:
            self.joystick_control_event = (joystick_toggle and not self.previous_joystick_toggle)

        if not self.calibrate_event:
            self.calibrate_event = (calibrate_toggle and not self.previous_calibrate_toggle)

        if not self.pid_control_event:
            self.pid_control_event = (pid_toggle and not self.previous_pid_control_toggle)

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_joystick_toggle = joystick_toggle
        self.previous_calibrate_toggle = calibrate_toggle
        self.previous_pid_control_toggle = pid_toggle

        # self.developing_command.trot_event = self.trot_event
        # self.developing_command.hop_event = self.hop_event
        # self.developing_command.joystick_control_event = self.joystick_control_event

        # Continuous Commands
        x_vel = max(min(self.config.x_vel_tf(command.y), self.config.max_x_velocity), -self.config.max_x_velocity)
        y_vel = max(min(self.config.y_vel_tf(command.x), self.config.max_y_velocity), -self.config.max_y_velocity)

        # x_vel = command.y * self.config.max_x_velocity  # ly
        # y_vel = command.x * self.config.max_y_velocity  # lx
        self.developing_command.horizontal_velocity = np.round(np.array([x_vel, y_vel]), self.rounding_dp)

        # Attitude
        self.developing_command.roll_command = command.roll_movement
        self.developing_command.pitch_command = command.pitch
        self.developing_command.yaw_command = command.yaw_rate
        self.developing_command.height_command = command.height_movement

        self.new_command = self.developing_command


    def get_command(self, state, message_rate):

        self.current_command = self.new_command

        self.current_command.trot_event = self.trot_event
        self.current_command.hop_event = self.hop_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.current_command.calibrate = self.calibrate_event
        self.current_command.pid_control = self.pid_control_event
        self.hop_event = False
        self.trot_event = False
        self.joystick_control_event = False
        self.calibrate_event = False
        self.pid_control_event = False

        # message_dt = 1.0 / message_rate
        #
        # deadbanded_pitch = deadband(
        #     self.current_command.pitch, self.config.pitch_deadband
        # )
        # pitch_rate = clipped_first_order_filter(
        #     state.pitch,
        #     deadbanded_pitch,
        #     self.config.max_pitch_rate,
        #     self.config.pitch_time_constant,
        # )
        # self.current_command.pitch = np.clip(state.pitch + message_dt * pitch_rate, -0.35, 0.35)
        # self.current_command.height = np.clip(
        #     state.height - message_dt * self.config.z_speed * self.current_command.height_movement, -0.27, -0.08)
        # self.current_command.roll = np.clip(
        #     state.roll + message_dt * self.config.roll_speed * self.current_command.roll_movement, -0.3, 0.3)

        return self.current_command
