#!/usr/bin/env python

# Written by: William L
# Last Modified 15/08/2024

from barbudor_ina3221.full import *
import board
from enum import IntEnum

from typing import List


class SensorIdx(IntEnum):
    FR = 0
    FL = 1
    RR = 2
    RL = 3


class MotorChan(IntEnum):
    R = 1
    U = 2
    L = 3


class CurrentSensor:
    def __init__(self, addr: int, reg: int, mask: int, value: int,
                 channels: list = (1, 2, 3),
                 shunt_resistor: list = (0.01, 0.01, 0.01)):
        """
        Initialises a new current sensor
        :param addr: Address of the sensor
        :param reg: Register to write too
        :param mask: Masks for bits
        :param value: Values to write
        :param channels: Channels to enable
        :param shunt_resistor: Shunt resistor values in ohm.
        """
        self._current_sensor = INA3221(board.I2C(), i2c_addr=addr, shunt_resistor=shunt_resistor)
        self._current_sensor.update(reg=reg,
                                    mask=mask,
                                    value=value)

        self.channels = channels
        for channel in self.channels:
            self._current_sensor.enable_channel(channel)
            self._current_sensor.enable_channel(channel)
            self._current_sensor.enable_channel(channel)

    @property
    def battery_voltage(self):
        avg = 0
        for channel in self.channels:
            avg += self._current_sensor.bus_voltage(channel) + self._current_sensor.shunt_voltage(channel)

        return avg / len(self.channels)

    @property
    def load_voltage(self):
        avg = 0
        for channel in self.channels:
            avg += self._current_sensor.bus_voltage(channel)

        return avg / len(self.channels)

    def get_current(self, channel: int) -> float:
        return self._current_sensor.current(channel)

    def get_shunt_voltage(self, channel: int) -> float:
        return self._current_sensor.shunt_voltage(channel)


class LegCurrentSensors:
    def __init__(self, fr_addr: int, fl_addr: int, rr_addr: int, rl_addr: int,
                 reg=C_REG_CONFIG,
                 mask=C_AVERAGING_MASK |
                      C_VBUS_CONV_TIME_MASK |
                      C_SHUNT_CONV_TIME_MASK |
                      C_MODE_MASK,
                 value=C_AVERAGING_16_SAMPLES |
                       C_VBUS_CONV_TIME_8MS |
                       C_SHUNT_CONV_TIME_8MS |
                       C_MODE_SHUNT_AND_BUS_CONTINOUS
                 ):
        self._current_sensors: List[CurrentSensor] = [CurrentSensor(fr_addr, reg=reg, mask=mask, value=value),
                                                      CurrentSensor(fl_addr, reg=reg, mask=mask, value=value),
                                                      CurrentSensor(rr_addr, reg=reg, mask=mask, value=value),
                                                      CurrentSensor(rl_addr, reg=reg, mask=mask, value=value)]

    @property
    def battery_voltage(self):
        avg = 0
        for current_sensor in self._current_sensors:
            avg += current_sensor.battery_voltage

        return avg / 4

    @property
    def load_voltage(self):
        avg = 0
        for current_sensor in self._current_sensors:
            avg += current_sensor.load_voltage

        return avg / 4

    @property
    def get_current(self):
        total = 0
        for current_sensor in self._current_sensors:
            for channel in current_sensor.channels:
                total += current_sensor.get_current(channel)

        return total

    def get_shunt_current(self, idx: int, channel: int) -> float:
        return float(self._current_sensors[idx].get_current(channel))

    def get_shunt_voltage(self, idx: int, channel: int) -> float:
        return self._current_sensors[idx].get_shunt_voltage(channel)

if __name__ == '__main__':
    string = "fr u"
    items = string.split(" ")
    print(MotorChan[items[1].upper()].value)