#!/usr/bin/env python
# coding: utf-8
import sys
import yaml
import numpy as np

from ..src.kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration

class CalibrateServo:
    #  0  [[front_right_hip, front_left_hip, back_right_hip, back_left_hip]
    #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
    #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]]

    def __init__(self):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.servo_interface = ServoInterface(Leg_linkage(Configuration()))
        self.servo_angles = np.zeros((3, 4))

    def run(self):

        angles = {"fr r":0, "fr u":0, "fr l":90, "fl r":0, "fl u":0, "fl l":90, "rr r":0, "rr u":0, "rr l":90, "rl r":0, "rl u":0, "rl l":90}
        motor = "fr r"
        while motor != "q":
            motor = input(f"\n{list(angles.keys())}\nselect motor (q to exit): ")
            if motor == "q":
                break
            elif motor not in list(angles.keys()):
                print("Invalid motor selection")
                continue
            value = 0

            while value != "q":
                value = input(f"\tAngle (q to exit): ")
                angles[motor] = value
                servo_angles = [[angles["fr r"], angles["fl r"], angles["rr r"], angles["rl r"]],
                                [angles["fr u"], angles["fl u"], angles["rr l"], angles["rl r"]],
                                [angles["fr l"], angles["fl l"], angles["rr u"], angles["rl r"]]]

                if value.isnumeric():
                    print(servo_angles)
                    self.servo_interface.set_angle(angles)
                elif value != "q":
                    print("\tInvalid angle")
                    continue

        new_angles = {"fr": {"roll": angles["fr r"], "upper": angles["fr u"], "lower": angles["fr l"]},
                      "fl": {"roll": angles["fl r"], "upper": angles["fl u"], "lower": angles["fl l"]},
                      "rr": {"roll": angles["rr r"], "upper": angles["rr u"], "lower": angles["rr l"]},
                      "rl": {"roll": angles["rl r"], "upper": angles["rl u"], "lower": angles["rl l"]}
                      }
        yaml.dump(new_angles, open("calibrate_servo_angles.yaml", "w"))




if __name__ == "__main__":
    calibrate_ob = CalibrateServo()
    calibrate_ob.run()