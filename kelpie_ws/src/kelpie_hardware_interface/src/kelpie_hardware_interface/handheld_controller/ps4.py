import numpy as np
import rospy
from kelpie_control.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy
from kelpie.msg import commands

class Ps4Interface:
    def __init__(self, config):
        self.config = config
        self.rounding_dp = 2

        self.noderate = rospy.get_param("noderate", 50.0)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.input_callback)
        self.command_pub = rospy.Publisher("/command_input", commands)


    def input_callback(self, msg):
        command = commands()

        command.gait_toggle = msg.buttons[5]  # R1
        command.hop_toggle = msg.buttons[0]  # x
        command.joystick_toggle = msg.buttons[4]  # L1
        command.x = deadband(msg.axes[1], self.config.pitch_deadband)  # ly
        command.y = deadband(msg.axes[0], self.config.pitch_deadband)  # lx
        command.pitch = deadband(msg.axes[4], self.config.pitch_deadband)  # ry
        command.yaw = deadband(msg.axes[3], self.config.pitch_deadband)  # rx
        command.height_movement = msg.axes[7]  # dpady
        command.roll = msg.axes[6]  # dpadx

        self.command_pub.publish(command)

