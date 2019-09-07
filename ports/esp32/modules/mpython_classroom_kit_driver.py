# labplus mPython-box library
# MIT license; Copyright (c) 2018 labplus

# mpython-box buildin periphers drivers

# history:
# V1.0 zhaohuijiang 

from machine import Pin, UART
import time, ujson
from time import sleep_ms, sleep_us, sleep

# touchpad
class BS8112A(object):
   """  """

   def __init__(self,i2c):
      self.addr = 80
      # config
      self._i2c = i2c
      self.config = [0xB0, 0x00, 0x00, 0x83, 0xf3, 0x98, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x00] 
      checksum = 0
      for i in range(1, 19):
         checksum += self.config[i]
      checksum &= 0xff
      self.config[18] = checksum
      # print(self.config[18])
      retry = 0
      if (retry < 5):
         try:
            self._i2c.writeto(self.addr, bytearray(self.config), True) 
            return
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

       # i2c.writeto(self.addr, b'\xB0', False) 
       # time.sleep_ms(10)
       # print(i2c.readfrom(self.addr, 17, True))

   # key map:
   # value       bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0 
   # bs8112a key Key8 Key7 Key6 Key5 Key4 Key3 Key2 Key1
   # mpython key       N    O    H    T    Y    P
   def key_value(self):
      retry = 0
      if (retry < 5):
         try:
            self._i2c.writeto(self.addr, b'\x08', False)  
            time.sleep_ms(10)
            value = self._i2c.readfrom(self.addr, 1, True)
            time.sleep_ms(10)
            return value
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

class Codec_mode():
   ES_MODULE_ADC_DAC  = 0x00      
   ES_MODULE_DAC  = 0x01    
   ES_MODULE_ADC  = 0x02     

class Es8388():
   """  """

   def __init__(self,i2c, adc_volume = 0, dac_volume  = 0, volume = 65):
      self._i2c=i2c
      self.addr = 16
      self.adc_volume = adc_volume
      self.dac_volume = dac_volume
      self.volume = volume
      self.set_voice_mute(1)
      retry = 0
      if (retry < 5):
         try:
            # i2c.writeto(self.addr, bytearray([0x19, 0x04])) # ES8388_DACCONTROL3 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
            #  Chip Control and Power Management 
            self._i2c.writeto(self.addr, bytearray([0x01, 0x50])) # ES8388_CONTROL2 0x40? 
            self._i2c.writeto(self.addr, bytearray([0x02, 0x00])) # ES8388_CHIPPOWER normal all and power up all
            self._i2c.writeto(self.addr, bytearray([0x08, 0x00])) # ES8388_MASTERMODE CODEC IN I2S SLAVE MODE 0x00: slave
            # dac setup
            self._i2c.writeto(self.addr, bytearray([0x04, 0xC0])) # ES8388_DACPOWER . disable DAC and disable Lout/Rout/1/2
            self._i2c.writeto(self.addr, bytearray([0x00, 0x12])) # ES8388_CONTROL1. Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
            self._i2c.writeto(self.addr, bytearray([0x17, 0x18])) # ES8388_DACCONTROL1 1a 0x18:16bit iis , 0x00:24
            self._i2c.writeto(self.addr, bytearray([0x18, 0x02])) # ES8388_DACCONTROL2 DACFsMode,SINGLE SPEED; DACFsRatio,256
            self._i2c.writeto(self.addr, bytearray([0x26, 0x00])) # ES8388_DACCONTROL16 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
            self._i2c.writeto(self.addr, bytearray([0x27, 0x90])) # ES8388_DACCONTROL17 only left DAC to left mixer enable 0db
            self._i2c.writeto(self.addr, bytearray([0x2a, 0x90])) # ES8388_DACCONTROL20 only right DAC to right mixer enable 0db
            self._i2c.writeto(self.addr, bytearray([0x2b, 0x80])) # ES8388_DACCONTROL21 set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
            self._i2c.writeto(self.addr, bytearray([0x2d, 0x00])) # ES8388_DACCONTROL23 vroi=0
            self.set_adc_dac_volume(Codec_mode.ES_MODULE_DAC, self.dac_volume, 0) #0db
            self._i2c.writeto(self.addr, bytearray([0x04, 0x3c])) # ES8388_DACPOWER 0x3c Enable DAC and Enable Lout/Rout/1/2
            # adc setup
            self._i2c.writeto(self.addr, bytearray([0x03, 0xff])) # ES8388_ADCPOWER
            self._i2c.writeto(self.addr, bytearray([0x09, 0xbb])) # ES8388_ADCCONTROL1 MIC Left and Right channel PGA gain
            self._i2c.writeto(self.addr, bytearray([0x0a, 0x00])) # ES8388_ADCCONTROL2 0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
            self._i2c.writeto(self.addr, bytearray([0x0b, 0x02])) # ES8388_ADCCONTROL3 clock input
            self._i2c.writeto(self.addr, bytearray([0x0c, 0x0c])) # ES8388_ADCCONTROL4 Left/Right data, Left/Right justified mode, Bits length 16bit, I2S format 0x0c?
            self._i2c.writeto(self.addr, bytearray([0x0d, 0x02])) # ES8388_ADCCONTROL5 ADCFsMode,singel SPEED,RATIO=256
            # ALC for Microphone
            self.set_adc_dac_volume(Codec_mode.ES_MODULE_ADC, self.adc_volume, 0) #0db
            self._i2c.writeto(self.addr, bytearray([0x03, 0x09])) # ES8388_ADCPOWER Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode
            # set volume
            self.set_volume(self.volume)
            self.set_voice_mute(0)
            # test 
            # for i in range(0, 52):
            #     i2c.writeto(self.addr, bytearray([i]))
            #     print("%d: %d" % (i, i2c.readfrom(self.addr, 1)[0]))
            return
         except:
               retry = retry + 1
      else:
         raise Exception("es8388 i2c read/write error!")

   def deinit(self):
      retry = 0
      if (retry < 5):
         try:
            self._i2c.writeto(self.addr, bytearray([0x02, 0xff])) # ES8388_CHIPPOWER reset and stop es838
            return
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

   def set_adc_dac_volume(self, mode, volume, dot):
      _volume = volume
      if (_volume < -96):
         _volume = -96
      else :
         _volume = 0
      _dot = 0
      if dot >= 5:
         _dot = 1

      _volume = (-_volume << 1) + _dot
      retry = 0
      if (retry < 5):
         try:
            if (mode == Codec_mode.ES_MODULE_ADC or mode == Codec_mode.ES_MODULE_ADC_DAC) :
               self._i2c.writeto(self.addr, bytearray([0x10, _volume])) # ES8388_ADCCONTROL8
               self._i2c.writeto(self.addr, bytearray([0x11, _volume])) # ES8388_ADCCONTROL9
            if (mode == Codec_mode.ES_MODULE_DAC or mode == Codec_mode.ES_MODULE_ADC_DAC) :
               self._i2c.writeto(self.addr, bytearray([0x1b, _volume])) # ES8388_DACCONTROL5
               self._i2c.writeto(self.addr, bytearray([0x1a, _volume])) # ES8388_DACCONTROL4
            return
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

   def set_volume(self, volume):
      self.volume = volume
      if (self.volume < 0):
         self.volume = 0
      elif (self.volume > 100):
         self.volume = 100

      retry = 0
      if (retry < 5):
         try:
            self._i2c.writeto(self.addr, bytearray([0x2e, self.volume//3])) # ES8388_DACCONTROL24
            self._i2c.writeto(self.addr, bytearray([0x2f, self.volume//3])) # ES8388_DACCONTROL25
            self._i2c.writeto(self.addr, bytearray([0x30, 0])) # ES8388_DACCONTROL26
            self._i2c.writeto(self.addr, bytearray([0x31, 0])) # ES8388_DACCONTROL27
            # print("volume L: %d" % (self.volume//3))
            return
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

   def get_volume(self):
         return self.volume

   def set_voice_mute(self, mute):
      retry = 0
      if (retry < 5):
         try:
            self._i2c.writeto(self.addr, b'\x19')  
            dac_ctr3 = self._i2c.readfrom(self.addr, 1)[0]
            if(mute):
               dac_ctr3 |= 0x04
            else:
               dac_ctr3 &= 0xFB
            self._i2c.writeto(self.addr, bytearray([0x19, dac_ctr3]))
         except:
               retry = retry + 1
      else:
         raise Exception("bs8112a i2c read/write error!")

uart2 = UART(2, baudrate=115200, rx=Pin.P8, tx=Pin.P23, timeout=50, timeout_char = 1024, rxbuf = 1024, txbuf = 256)

class K210Error(Exception):
   """K210异常类"""
   pass
 


class K210():
   def __init__(self):
        t1 = time.ticks_ms()
        while (time.ticks_diff(time.ticks_ms(), t1) < 10000):
            rsp = self.send_cmd({'GET_KEYS':0}) #通过发获取按键指令测试K210是否初始化成功
            if rsp and isinstance(rsp, dict):
                for key,value in rsp.items():
                    if key == 'RET_KEYS': 
                        return
        raise K210Error("K210 init failed!")

   def send_cmd(self, command, wait=True, timeout=500):
      json_stream = ujson.dumps(command)
      uart2.write(json_stream + '\n')
      # print("UART_Send:%s" % (json_stream + '\n'))
      t1 = time.ticks_ms()
      while wait:
         if uart2.any() > 0:
               r = uart2.readline()
               # print("UART_Recv:%s" % r)
               try:
                  rsp = ujson.loads(r)
                  # print("rsp:%s" % rsp)
               except Exception as e:
                  print(e)
                  break  
               else :
                  if rsp and isinstance(rsp, dict):
                     for key, value in rsp.items():
                        if key == 'ERROR':
                           raise K210Error(value)
                     return rsp
         if time.ticks_diff(time.ticks_ms(), t1) > timeout:
            # raise K210Error("k210 not respone!")
            break
      return None

   def get_key(self):
      rsp = self.send_cmd({'GET_KEYS':0})
      if rsp and isinstance(rsp, dict):
            for key,value in rsp.items():
               if key == 'RET_KEYS': 
                  return value
      return None

   def get_distance(self):
      rsp = self.send_cmd({'GET_DISTANCE':0})
      if rsp and isinstance(rsp, dict):
            for key,value in rsp.items():
               if key == 'RET_DISTANCE': 
                  return value
      return None

   def set_motor(self, speed):
      rsp = self.send_cmd({'SET_MOTOR':speed})
      if rsp and isinstance(rsp, dict):
         for key,value in rsp.items():
               if key == 'RET_MOTOR': 
                  return value
      return None


   def select_model(self,*args):

      self.send_cmd({'SELE_MOD':args[0]})

   def load_model(self,**kws):

      self.send_cmd({'LOD_MOD':kws})

   def detect_yolo(self):
      rsp = self.send_cmd({'DET_YO':0})
      if rsp and isinstance(rsp, dict):
         for key,value in rsp.items():
            if key == 'RET_DET_YO': 
               return value
      return None

   def predict_net(self):
      rsp = self.send_cmd({'PRE_NET':0})
      if rsp and isinstance(rsp, dict):
         for key,value in rsp.items():
            if key == 'RET_PRE_NET': 
               return value
      return None

   def deinit_yolo(self):
      self.send_cmd({'DINT_YO':0})

   def camera_snapshot(self,*arg):
      self.send_cmd({'SNAPSHOT':0})

   def lcd_init(self,**kws):
      self.send_cmd({'LCD_INT':kws})

   def lcd_display(self,**kws):
      self.send_cmd({'LCD_DISP':kws})

   def lcd_clear(self,**kws):
      self.send_cmd({'LCD_CLR':kws})

   def lcd_draw_string(self,*args):
      self.send_cmd({'LCD_STR':args})
