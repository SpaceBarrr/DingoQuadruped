#!/usr/bin/env python

#ManhLinhPhan
import rospy
import math
from sensor_msgs.msg import JoyFeedbackArray
from kelpie.msg import commands
from pydualsense import *
from pydualsense.enums import ConnectionType


class Ds5Ros():
    def __init__(self):

        self.logging_prefix = "DS5_ROS: "

        # 0 is startup, 1 is running
        self.node_state = 0

        # create dualsense
        self.dualsense = pydualsense()

        self.noderate = rospy.get_param("noderate", 50.0)

        # receive feedback from robot_joy_control node 
        self.joy_sub_topic = rospy.get_param("joy_sub", "joy/set_feedback")

        # controller is not straight zero for the axis. prevent tiny robot movements
        self.deadband = rospy.get_param("deadzone", 10)

        self.joy_sub = rospy.Subscriber(self.joy_sub_topic, JoyFeedbackArray, self.set_feedback)
        self.command_pub = rospy.Publisher("/command_input", commands, queue_size=0)

        self.maskR = 0xFF0000
        self.maskG = 0x00FF00
        self.maskB = 0x0000FF

    '''
    JoyFeedback Struct
    TYPE_LED = 0 | TYPE_RUMBLE = 1 (use also for feedback on rear button) | TYPE_BUZZER = 2
    type    : above
    id      : 0 = Left Motor Rumble | 1 = Right Motor Rumble | 2 = L2 feedback Trigger | 3 = R2 feedback Trigger
    Using combination of type and id to get the result
    intensity:  
    '''

    def set_feedback(self, msg):
        #print(msg)
        for feedback in msg.array:  #not iterable
            # type = 0: LED control
            # RGB (24bit) is sent as float32 -> can almost be covered by 23 mantissa  
            if feedback.type == 0:
                int_intensity = int(feedback.intensity)
                light_red = (int_intensity & self.maskR) >> 16
                light_green = (int_intensity & self.maskG) >> 8
                light_blue = (int_intensity & self.maskB)

                self.dualsense.light.setColorI(light_red, light_green, light_blue)
                continue

            #other type
            if feedback.intensity > 1.0 or feedback.intensity < 0.0:
                rospy.logerr(self.logging_prefix + 'intensity must be in range 0.0 - 1.0')
                continue
            else:
                feedback.intensity = feedback.intensity * 255.0

            #intensity muss be an integer
            if feedback.type == 1 and feedback.id == 0:
                pass  #add effect later
            elif feedback.type == 1 and feedback.id == 1:
                pass  #add effect later
            elif feedback.type == 1 and feedback.id == 2:
                self.dualsense.triggerL.setMode(TriggerModes.Rigid)
                self.dualsense.triggerL.setForce(1, int(feedback.intensity))
            elif feedback.type == 1 and feedback.id == 3:
                self.dualsense.triggerR.setMode(TriggerModes.Rigid)
                self.dualsense.triggerR.setForce(1, int(feedback.intensity))

    def command_publish(self):
        # try:
        command = commands()

        # Buttons options for PS5
        # self.dualsense.state.cross
        # self.dualsense.state.circle
        # self.dualsense.state.triangle
        # self.dualsense.state.square
        #
        # self.dualsense.state.DpadUp
        # self.dualsense.state.DpadRight
        # self.dualsense.state.DpadDown
        # self.dualsense.state.DpadLeft
        #


        # self.dualsense.state.L1
        # self.dualsense.state.L2Btn    # L2 boolean button
        # self.dualsense.state.L2       # L2 analog values
        # self.dualsense.state.L3       # Left stick
        # self.dualsense.state.R1
        # self.dualsense.state.R2Btn    # R2 boolean button
        # self.dualsense.state.R2       # R2 analog values
        # self.dualsense.state.R3       # Right stick
        #
        # self.dualsense.state.ps
        # self.dualsense.state.share
        # self.dualsense.state.options
        # self.dualsense.state.touchBtn
        # self.dualsense.state.LX/128.0         (L = Left, X = Left-right direction)
        # self.dualsense.state.LY/128.0         (L = Left, Y = Up-Down direction)
        # self.dualsense.state.RX/128.0         (R = Right, X = Left-right direction)
        # self.dualsense.state.RY/128.0         (R = Right, Y = Up-Down direction)

        command.y = self.apply_deadband(self.dualsense.state.LY)/128        # Scale input to 0..1
        command.x = self.apply_deadband(self.dualsense.state.LX)/128
        command.yaw = self.apply_deadband(self.dualsense.state.RX)/128
        command.yaw = self.apply_deadband(self.dualsense.state.RX)/128
        command.roll = 0
        command.pitch = 0

        command.gait_toggle = self.dualsense.state.L3
        command.hop_toggle = False
        command.joystick_toggle = self.dualsense.state.R3
        command.height_movement = 0
        print(command)

        # print(self.dualsense.state.L3)
        self.command_pub.publish(command)


    def main_loop(self):
        rate = rospy.Rate(self.noderate)

        ### initialize controller if possible, else wait

        # find device and initialize
        while not rospy.is_shutdown():
            if self.node_state == 0:
                try:
                    self.dualsense.init()
                except:
                    rospy.logwarn_throttle(2, "Cannot initialize controller!")
                else:
                    # set rgb led to green
                    self.dualsense.light.setColorI(0, 255, 0)
                    self.node_state = 1

            elif self.node_state == 1:
                # if error -> node_state = 0
                try:
                    #Add new attribute cable_connection in dualsense
                    #Init cable_connection = True
                    #Catch IOError in dualsense.writeReport and set cable_connection = False
                    if self.dualsense.determineConnectionType() == ConnectionType.USB:
                        # rospy.loginfo_throttle(2, "DS5_Ros is alive!")
                        self.command_publish()
                    else:
                        rospy.logerr("Lost connection! Go back to init!")
                        self.node_state = 0
                except:
                    pass

            rate.sleep()
        self.dualsense.close()
    def apply_deadband(self, value):
        self.deadband = 15
        if abs(value) < self.deadband:
            value = 0.0
        return value


if __name__ == '__main__':
    rospy.init_node("ps5")
    ds5 = Ds5Ros()
    ds5.main_loop()
