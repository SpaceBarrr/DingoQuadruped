#!/usr/bin/env python
# coding: utf-8
import yaml
import os
import curses
import time
import numpy as np

from kelpie_hardware_interface.servo.Interface import ServoInterface
from kelpie_common.Config import Leg_linkage, Configuration
from kelpie_common.Utilities import format_angles
from kelpie_hardware_interface.current_sense.current_sensor import LegCurrentSensors, SensorIdx, MotorChan

DIR_PATH = os.path.dirname(os.path.realpath(__file__))





if __name__ == "__main__":
    calibrate_ob = CalibrateServo()
    calibrate_ob.run()
