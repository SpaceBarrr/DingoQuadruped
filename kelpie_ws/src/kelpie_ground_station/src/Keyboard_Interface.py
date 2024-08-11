import os
import signal
import sys

import rospy
from pynput import keyboard
from kelpie.msg import commands


class Keyboard:
    def __init__(self):
        self.used_keys = ['w', 'a', 's', 'd', '1', '2', '7', '8', '9', '0', keyboard.Key.shift, keyboard.Key.backspace,
                          keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()

        self.noderate = rospy.get_param("noderate", 50.0)
        self.command_pub = rospy.Publisher("/command_input", commands, queue_size=0)
        self.command = commands()

        self.speed_multiplier = 1


    def on_press(self, key):
        if hasattr(key, 'char'):
            key = key.char

        # increase speed
        if key == keyboard.Key.shift:
            self.speed_multiplier = 2
        # cartesian movements (x-y plane)
        elif key == 'w' or key == 'W':
            self.command.x = 0.5 * self.speed_multiplier
        elif key == 's' or key == 'S':
            self.command.x = -0.5 * self.speed_multiplier
        elif key == 'a' or key == 'A':
            self.command.y = 0.5 * self.speed_multiplier
        elif key == 'd' or key == 'D':
            self.command.y = -0.5 * self.speed_multiplier
        # pitch, yaw, roll, height
        elif key == keyboard.Key.up:
            self.command.pitch = 0.5 * self.speed_multiplier
        elif key == keyboard.Key.down:
            self.command.pitch = -0.5 * self.speed_multiplier
        elif key == keyboard.Key.left:
            self.command.yaw = 0.5 * self.speed_multiplier
        elif key == keyboard.Key.right:
            self.command.yaw = -0.5 * self.speed_multiplier
        elif key == '8':
            self.command.roll = 1
        elif key == '7':
            self.command.roll = -1
        elif key == '0':
            self.command.height_movement = 1
        elif key == '9':
            self.command.height_movement = -1
        # mode settings
        elif key == '1':
            self.command.gait_toggle = True
        elif key == '2':
            self.command.hop_toggle = True
        elif key == keyboard.Key.backspace:
            self.command.joystick_toggle = True


    def on_release(self, key):
        if hasattr(key, 'char'):
            key = key.char

        # cartesian movements (x-y plane)
        if key == keyboard.Key.shift:
            self.speed_multiplier = 1
        elif key == 'w' or key == 'W':
            self.command.x = 0.0
        elif key == 's' or key == 'S':
            self.command.x = 0.0
        elif key == 'a' or key == 'A':
            self.command.y = 0.0
        elif key == 'd' or key == 'D':
            self.command.y = 0.0
        # pitch, yaw, roll, height
        elif key == keyboard.Key.up:
            self.command.pitch = 0.0
        elif key == keyboard.Key.down:
            self.command.pitch = 0.0
        elif key == keyboard.Key.left:
            self.command.yaw = 0.0
        elif key == keyboard.Key.right:
            self.command.yaw = 0.0
        elif key == '8':
            self.command.roll = 1
        elif key == '7':
            self.command.roll = -1
        elif key == '0':
            self.command.height_movement = 1
        elif key == '9':
            self.command.height_movement = -1
        # mode settings
        elif key == '1':
            self.command.gait_toggle = False
        elif key == '2':
            self.command.hop_toggle = False
        elif key == keyboard.Key.backspace:
            self.command.joystick_toggle = False


    def command_publish(self):
        self.command_pub.publish(self.command)


    def signal_handler(sig, frame):
        sys.exit(0)


    def main_loop(self):
        rate = rospy.Rate(self.noderate)

        if os.getenv("DISPLAY", default="-") != "-":
            rospy.logfatal(
                "This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
            rospy.sleep(1)
            sys.exit(0)

        signal.signal(signal.SIGINT, self.signal_handler)

        while not rospy.is_shutdown():
            self.command_publish()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("keyboard_input_listener")
    keyboard_listener = Keyboard()
    keyboard_listener.main_loop()
