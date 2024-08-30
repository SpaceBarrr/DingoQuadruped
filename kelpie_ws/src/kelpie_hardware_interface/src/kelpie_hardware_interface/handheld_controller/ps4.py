#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from kelpie.msg import commands

class Ps4Interface:
    def __init__(self):
        self.rounding_dp = 2
        self.deadband = 0.1

        self.noderate = rospy.get_param("noderate", 50.0)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.input_callback)
        self.command_pub = rospy.Publisher("/kelpie/command_input", commands, queue_size=1)


    def input_callback(self, msg):
        command = commands()

        command.gait_toggle = msg.buttons[5]  # R1
        command.hop_toggle = msg.buttons[0]  # x
        command.joystick_toggle = msg.buttons[4]  # L1
        command.x = self.apply_deadband(msg.axes[0])  # ly
        command.y = self.apply_deadband(msg.axes[1])  # lx
        command.pitch = self.apply_deadband(msg.axes[4])  # ry
        command.yaw_rate = self.apply_deadband(msg.axes[3])  # rx
        command.height_movement = msg.axes[7]  # dpady
        command.roll_movement = msg.axes[6]  # dpadx

        self.command_pub.publish(command)
        print(command)



    def apply_deadband(self, value):
        if abs(value) < self.deadband:
            value = 0.0
        return value

if __name__ == "__main__":
    rospy.init_node("ps4")
    ps4 = Ps4Interface()


    rospy.spin()