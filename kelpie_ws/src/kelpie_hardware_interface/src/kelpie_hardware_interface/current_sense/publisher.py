#!/usr/bin/env python
# coding: utf-8
from kelpie_hardware_interface.current_sense.current_sensor import LegCurrentSensors, SensorIdx, MotorChan
# from current_sensor import LegCurrentSensors, SensorIdx, MotorChan
from kelpie.msg import joint_states
from kelpie.msg import leg_state
from kelpie_common.Utilities import build_leg_msg

CURRENT_MSG = joint_states()
FR_STATE_MSG = leg_state()
FL_STATE_MSG = leg_state()
RR_STATE_MSG = leg_state()
RL_STATE_MSG = leg_state()
CURRENT_SENSORS = LegCurrentSensors(fr_addr=0x43,  # 1000 0011
                                    fl_addr=0x41,  # 1000 0001
                                    rr_addr=0x42,  # 1000 0010
                                    rl_addr=0x40)  # 1000 0000
def get_currents(servo):
    sensor = SensorIdx[servo].value
    channel_r = MotorChan.R.value
    channel_u = MotorChan.U.value
    channel_l = MotorChan.L.value
    return (round(CURRENT_SENSORS.get_shunt_current(sensor, channel_r), 3),
            round(CURRENT_SENSORS.get_shunt_current(sensor, channel_u), 3),
            round(CURRENT_SENSORS.get_shunt_current(sensor, channel_l), 3))


def publish(publisher):
    CURRENT_MSG.fr = build_leg_msg(FR_STATE_MSG, get_currents("FR"))
    CURRENT_MSG.fl = build_leg_msg(FL_STATE_MSG, get_currents("FL"))
    CURRENT_MSG.rr = build_leg_msg(RR_STATE_MSG, get_currents("RR"))
    CURRENT_MSG.rl = build_leg_msg(RL_STATE_MSG, get_currents("RL"))
    publisher.publish(CURRENT_MSG)

