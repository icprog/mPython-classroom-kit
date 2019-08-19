# labplus mPython-box library
# MIT license; Copyright (c) 2018 labplus

# mpython-box buildin periphers drivers

# history:
# V1.0 zhaohuijiang 

from machine import Pin, ADC
import time, ujson
from mpython_classroom_kit_driver import K210,K210Error
from mpython import i2c
# human infrared
pir = Pin(21, mode = Pin.IN, pull = None)

# slide POT
slider_res =  ADC(Pin(34))
slider_res.atten(slider_res.ATTN_11DB)

k210 = K210()

def get_distance():
    """超声波,范围2~340cm"""
    return k210.get_distance()

def get_key():
    """方向按键,返回按键列表"""
    key_map = {0: 'left', 1: 'right', 2: 'up', 3: 'down', 4: 'ok'}
    key_set=set()
    _key = k210.get_key()
    for i in range(5):
        if (_key >> i) & 0x01 :
            key_set.add(key_map[i])
    return key_set

def set_motor(speed):
    """马达,范围±100"""
    if speed < -100 or speed > 100:
            raise K210Error("Invalid value,range in -100~100")
    k210.set_motor(speed)


class AI():
    def __init__(self):
        pass

    def set_kmodel(self):
        pass

    def recognize(self):
        return k210.ai_recognize()

    def monitor(self):
        return k210.monitor('')



