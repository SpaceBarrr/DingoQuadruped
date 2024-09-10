#!/usr/bin/env python
# coding: utf-8
from batt_sensor import BattVSensor
from kelpie.msg import xyz_float32

VOLTAGE_MSG = xyz_float32
BATT_SENSOR = BattVSensor()
def publish(publisher):
    VOLTAGE_MSG.x = BATT_SENSOR.v0_batt     # Cell 1 voltage
    VOLTAGE_MSG.y = BATT_SENSOR.v1_batt     # Cell 2 voltage
    VOLTAGE_MSG.z = BATT_SENSOR.v2 * 2      # Raw battery voltage
    publisher.publish(VOLTAGE_MSG)

