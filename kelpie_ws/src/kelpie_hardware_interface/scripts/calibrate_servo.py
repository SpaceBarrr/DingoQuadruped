#!/usr/bin/env python
# coding: utf-8
import yaml
import os
import curses
import time
import numpy as np
from threading import Thread
from kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration
from kelpie_common.current_sensor_calibrate import Calibrator
from kelpie_common.Utilities import format_angles
from kelpie_hardware_interface.current_sense.current_sensor import LegCurrentSensors, SensorIdx, MotorChan
from kelpie_common.Config import ServoIndex as s_idx

DIR_PATH = os.path.dirname(os.path.realpath(__file__))



class MotorCurrents(Thread):
    def __init__(self, *args, **kwargs) -> None:
        """
        Initialise the ImuSubscriber class
        """
        super().__init__(*args, **kwargs)
        self.currents = np.zeros((3, 4))
        self.current_sensors = LegCurrentSensors(fr_addr=0x40,
                                                 fl_addr=0x42,
                                                 rr_addr=0x41,
                                                 rl_addr=0x43)


    def run(self):
        # Keep pointer/reference. Do not overwrite with new class reference.
        while True:
            self.currents[:, 0] = self.get_currents("FR")
            self.currents[:, 1] = self.get_currents("FL")
            self.currents[:, 2] = self.get_currents("RR")
            self.currents[:, 3] = self.get_currents("RL")
            time.sleep(0.1)

    def get_currents(self, servo):
        sensor = SensorIdx[servo].value
        channel_r = MotorChan.R.value
        channel_u = MotorChan.U.value
        channel_l = MotorChan.L.value
        return (round(self.current_sensors.get_shunt_current(sensor, channel_r), 3),
                round(self.current_sensors.get_shunt_current(sensor, channel_u), 3),
                round(self.current_sensors.get_shunt_current(sensor, channel_l), 3))

    def get_current(self, servo):
        return self.currents[s_idx[servo].value]

class CalibrateServo:
    ''' 
    0  [[front_right_hip, front_left_hip, back_right_hip, back_left_hip]
    1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
    2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]]
    '''

    def __init__(self):
        self.pwm_max = 2400
        self.pwm_min = 370
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

        self.leg_currents = LegCurrentSensors(fr_addr=0x43, # 1000 0011
                                              fl_addr=0x41, # 1000 0001
                                              rr_addr=0x42, # 1000 0010
                                              rl_addr=0x40) # 1000 0000

        self.motor_currents = MotorCurrents()
        self.motor_currents.start()
        self.auto_calibrator = Calibrator(self.motor_currents)

        self.servo_interface = ServoInterface(Leg_linkage(Configuration()))
        self.servo_interface.physical_calibration_offsets = np.zeros((3, 4))
        self.auto_calibrator.servo_interface = self.servo_interface
        self.auto_calibrator.servo_angles = self.servo_angles

    def _set_start_pos(self):
        self.servo_interface.physical_calibration_offsets = format_angles(self.offset)
        self.servo_angles = format_angles(self.ANGLES)
        self.servo_interface.set_servo_angles(self.servo_angles)

    def run(self, current):
        # while True:
        #     print(current.currents)
        stdscr = curses.initscr()
        stdscr.clear()

        curses.noecho()
        curses.cbreak()

        motor = "fr r"

        while motor != "q":
            motor = self.str_input(stdscr,
                                   0,
                                   0,
                                   f"\n{list(self.offset.keys())}\nselect motor (q to exit): ",
                                   line_offset=3).decode("utf-8").lower()

            if motor == "q":
                break
            elif motor.lower() == "relax":
                self.servo_interface.relax_all_motors()
                continue
            elif motor.lower() == "auto":
                self.auto_calibrator.run(format_angles(self.offset) )
                continue
            elif motor not in list(self.offset.keys()):
                stdscr.addstr(3, 0, "Invalid motor selection")
                stdscr.refresh()
                time.sleep(0.5)
                stdscr.clear()
                continue
            else:
                stdscr.nodelay(True)
                curses.noecho()
                stdscr.clear()

                motor_enums = motor.split(" ")
                sensor = SensorIdx[motor_enums[0].upper()].value
                channel = MotorChan[motor_enums[1].upper()].value

            stdscr.refresh()
            value = 0

            while value != "q":
                
                stdscr.addstr(0, 0, f"Use 'w' and 's' to change servo angles")
                stdscr.addstr(1, 0, f"Current angle: {self.offset[motor]}")
                stdscr.addstr(2, 0, f"Servo Current: {round(self.leg_currents.get_shunt_current(sensor, channel), 3)}")
                stdscr.addstr(3, 0, "")
                value = stdscr.getch()
                stdscr.refresh()
                if value == -1:
                    continue

                if value == ord("q"):
                    stdscr.nodelay(False)
                    stdscr.clear()
                    break

                elif value == ord("w"):
                    self.offset[motor] += 1

                elif value == ord("s"):
                    self.offset[motor] -= 1

                self.servo_interface.physical_calibration_offsets = format_angles(self.offset)
                self.servo_interface.set_servo_angles(self.servo_angles)
                curses.flushinp()
                time.sleep(0.04)

        new_angles = {"fr": {"roll": self.offset["fr r"], "upper": self.offset["fr u"], "lower": self.offset["fr l"]},
                      "fl": {"roll": self.offset["fl r"], "upper": self.offset["fl u"], "lower": self.offset["fl l"]},
                      "rr": {"roll": self.offset["rr r"], "upper": self.offset["rr u"], "lower": self.offset["rr l"]},
                      "rl": {"roll": self.offset["rl r"], "upper": self.offset["rl u"], "lower": self.offset["rl l"]}
                      }
        yaml.dump(new_angles, open(f"{DIR_PATH}/calibrate_servo_angles.yaml", "w+"))

    @staticmethod
    def str_input(stdscr, r, c, prompt_string, line_offset=1):
        curses.echo()
        stdscr.addstr(r, c, prompt_string)
        stdscr.refresh()
        input = stdscr.getstr(r + line_offset, c, 20)
        return input  # ^^^^  reading input at next line


if __name__ == "__main__":
    current = MotorCurrents()
    current.start()
    calibrate_ob = CalibrateServo()
    calibrate_ob.run(current)
