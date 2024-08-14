#!/usr/bin/env python
# coding: utf-8
import yaml
import os
import numpy as np

from kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration

DIR_PATH = os.path.dirname(os.path.realpath(__file__))


class CalibrateServo:
    ''' 
    0  [[front_right_hip, front_left_hip, back_right_hip, back_left_hip]
    1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
    2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]]
    '''

    def __init__(self):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.servo_interface = ServoInterface(Leg_linkage(Configuration()))
        self.servo_interface.physical_calibration_offsets = np.zeros((3, 4))
        self.servo_angles = np.zeros((3, 4))
        self.ANGLES = {"fr r": 0, "fr u": 0, "fr l": 90, "fl r": 0, "fl u": 0, "fl l": 90, "rr r": 0, "rr u": 0,
                       "rr l": 90, "rl r": 0, "rl u": 0, "rl l": 90}
        if os.path.isfile(f"{DIR_PATH}/calibrate_servo_angles.yaml"):
            with open(f"{DIR_PATH}/calibrate_servo_angles.yaml") as stream:
                raw_angles = yaml.load(stream, Loader=yaml.SafeLoader)
            self.offset = {
                "fr r": raw_angles["fr"]["roll"], "fr u": raw_angles["fr"]["upper"], "fr l": raw_angles["fr"]["lower"],
                "fl r": raw_angles["fl"]["roll"], "fl u": raw_angles["fl"]["upper"], "fl l": raw_angles["fl"]["lower"],
                "rr r": raw_angles["rr"]["roll"], "rr u": raw_angles["rr"]["upper"], "rr l": raw_angles["rr"]["lower"],
                "rl r": raw_angles["rl"]["roll"], "rl u": raw_angles["rl"]["upper"], "rl l": raw_angles["rl"]["lower"]
            }
        else:
            print("Could not find calibrate_servo_angles.yaml - Using defaults...")
            self.offset = {"fr r": 0, "fr u": 0, "fr l": 0, "fl r": 0, "fl u": 0, "fl l": 0, "rr r": 0, "rr u": 0,
                           "rr l": 0, "rl r": 0, "rl u": 0, "rl l": 0}
        self._set_start_pos()

    def _set_start_pos(self):
        self.servo_interface.physical_calibration_offsets = self._format_offset()
        self.servo_angles = np.array(
            [[self.ANGLES["fr r"], self.ANGLES["fl r"], self.ANGLES["rr r"], self.ANGLES["rl r"]],
             [self.ANGLES["fr u"], self.ANGLES["fl u"], self.ANGLES["rr u"], self.ANGLES["rl u"]],
             [self.ANGLES["fr l"], self.ANGLES["fl l"], self.ANGLES["rr l"], self.ANGLES["rl l"]]])

        self.servo_interface.set_servo_angles(self.servo_angles)

    def _format_offset(self):
        return np.array([
            [self.offset["fr r"], self.offset["fl r"], self.offset["rr r"], self.offset["rl r"]],
            [self.offset["fr u"], self.offset["fl u"], self.offset["rr u"], self.offset["rl u"]],
            [self.offset["fr l"], self.offset["fl l"], self.offset["rr l"], self.offset["rl l"]]
        ])

    def run(self):
        motor = "fr r"
        while motor != "q":
            motor = input(f"\n{list(self.offset.keys())}\nselect motor (q to exit): ")
            if motor == "q":
                break
            elif motor.lower() == "relax":
                self.servo_interface.relax_all_motors()
                continue
            elif motor not in list(self.offset.keys()):
                print("Invalid motor selection")
                continue
            value = 0

            while value != "q":
                print(f"\tCurrent value: {self.offset[motor]}")
                value = input(f"\tAngle (q to exit): ")
                if value == "q":
                    break
                try:
                    self.offset[motor] = float(value)
                    self.servo_interface.physical_calibration_offsets = self._format_offset()
                    self.servo_interface.set_servo_angles(self.servo_angles)
                except ValueError as error:
                    print(f"\tInvalid angle: {error}")
                    continue

        new_angles = {"fr": {"roll": self.offset["fr r"], "upper": self.offset["fr u"], "lower": self.offset["fr l"]},
                      "fl": {"roll": self.offset["fl r"], "upper": self.offset["fl u"], "lower": self.offset["fl l"]},
                      "rr": {"roll": self.offset["rr r"], "upper": self.offset["rr u"], "lower": self.offset["rr l"]},
                      "rl": {"roll": self.offset["rl r"], "upper": self.offset["rl u"], "lower": self.offset["rl l"]}
                      }
        yaml.dump(new_angles, open(f"{DIR_PATH}/calibrate_servo_angles.yaml", "w+"))


if __name__ == "__main__":
    calibrate_ob = CalibrateServo()
    calibrate_ob.run()
