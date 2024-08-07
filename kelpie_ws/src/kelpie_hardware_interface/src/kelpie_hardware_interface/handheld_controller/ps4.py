import numpy as np
import rospy
from kelpie_control.Command import Command
from kelpie_control.State import BehaviorState
from kelpie_control.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy
from kelpie.msg import commands

class Ps4Interface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0

        self.rounding_dp = 2

        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0

        self.input_messages = rospy.Subscriber("joy", Joy, self.input_callback)
        self.output_commands = rospy.Publisher("/command_input", commands)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

    def input_callback(self, msg):
        gait_toggle = msg.buttons[5]  # R1
        hop_toggle = msg.buttons[0]  # x
        joystick_toggle = msg.buttons[4]  # L1
        x_vel = (msg.axes[1]) * self.config.max_x_velocity  # ly
        y_vel = msg.axes[0] * self.config.max_y_velocity  # lx
        self.developing_command.yaw_rate = np.round(msg.axes[3], self.rounding_dp) * self.config.max_yaw_rate  # rx
        self.developing_command.pitch = np.round(msg.axes[4], self.rounding_dp) * self.config.max_pitch  # ry
        self.developing_command.height_movement = np.round(msg.axes[7], self.rounding_dp)  # dpady
        self.developing_command.roll_movement = -np.round(msg.axes[6], self.rounding_dp)  # dpadx