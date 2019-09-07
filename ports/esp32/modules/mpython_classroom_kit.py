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
        if _key !=None and (_key >> i) & 0x01 :
            key_set.add(key_map[i])
    return key_set

def set_motor(speed):
    """马达,范围±100"""
    if speed < -100 or speed > 100:
            raise K210Error("Invalid value,range in -100~100")
    return k210.set_motor(speed)


class Model(object):

    def __init__(self):
        self.FACE_YOLO = 1
        self.CLASS_20_YOLO = 2
        self.MNIST_NET = 3
        self.CLASS_1000_NET =4
        self.YOLO2=1
        self.MOBILENET=2

    def select_model(self, builtin=None):
        """内置模型选择"""
        k210.select_model(builtin)

    def load_model(self, path,model_type,classes, anchor=None):
        """加载外部模型"""
        k210.load_model(path=path,model_type=model_type,classes=classes,anchor=anchor)

    def detect_yolo(self):
        """yolo模型应用"""
        return k210.detect_yolo()

    def deinit_yolo(self):
        """模型释放"""
        k210.deinit_yolo()

    def predict_net(self):
        """MobileNet模型预测"""
        return k210.predict_net()


class LCD(object):
    BLACK = 0
    NAVY = 15
    DARKGREEN = 992
    DARKCYAN = 1007
    MAROON = 30720
    PURPLE = 30735
    OLIVE = 31712
    LIGHTGREY = 50712
    DARKGREY = 31727
    BLUE = 31
    GREEN = 2016
    RED = 63488
    MAGENTA = 63519
    YELLOW = 65504
    WHITE = 65535
    ORANGE = 64800
    GREENYELLOW = 45029
    PINK = 63519

    def init(self,freq=15000000,color = 0):
        k210.lcd_init(freq=freq,color = color)

    def display(self,oft=(0,0),roi=(0,0,320,240)):
        k210.lcd_display(oft=oft,roi=roi)

    def clear(self,color=0):
        k210.lcd_clear(color=color)

    def draw_string(self,*args):
        k210.lcd_draw_string(*args)

class Camera(object):

    def snapshot(self):
        k210.camera_snapshot()

    
