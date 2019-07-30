# labplus mPython library
# MIT license; Copyright (c) 2018 labplus
# V1.0 Zhang KaiHua(apple_eat@126.com)

# mpython buildin periphers drivers

# history:
# V1.1 add oled draw function,add buzz.freq().  by tangliufeng
# V1.2 add servo/ui class,by tangliufeng

from machine import I2C, PWM, Pin, ADC, TouchPad, UART
from ssd1106 import SSD1106_I2C
import esp, math, time, network
import ustruct, array
from neopixel import NeoPixel
from esp import dht_readinto
from time import sleep_ms, sleep_us, sleep
from framebuf import FrameBuffer

i2c = I2C(scl=Pin(Pin.P19), sda=Pin(Pin.P20), freq=400000)

class Font(object):
    def __init__(self, font_address=0x400000):
        self.font_address = font_address
        buffer = bytearray(18)
        esp.flash_read(self.font_address, buffer)
        self.header, \
            self.height, \
            self.width, \
            self.baseline, \
            self.x_height, \
            self.Y_height, \
            self.first_char,\
            self.last_char = ustruct.unpack('4sHHHHHHH', buffer)
        self.first_char_info_address = self.font_address + 18

    def GetCharacterData(self, c):
        uni = ord(c)
        # if uni not in range(self.first_char, self.last_char):
        #     return None
        if (uni < self.first_char or uni > self.last_char):
            return None
        char_info_address = self.first_char_info_address + \
            (uni - self.first_char) * 6
        buffer = bytearray(6)
        esp.flash_read(char_info_address, buffer)
        ptr_char_data, len = ustruct.unpack('IH', buffer)
        if (ptr_char_data) == 0 or (len == 0):
            return None
        buffer = bytearray(len)
        esp.flash_read(ptr_char_data + self.font_address, buffer)
        return buffer

class TextMode():
    normal = 1
    rev = 2
    trans = 3
    xor = 4

class OLED(SSD1106_I2C):
    """ 128x64 oled display """

    def __init__(self):
        super().__init__(128, 64, i2c)
        self.f = Font()
        if self.f is None:
            raise Exception('font load failed')

    def DispChar(self, s, x, y, mode=TextMode.normal):
        if self.f is None:
            return
        for c in s:
            data = self.f.GetCharacterData(c)
            if data is None:
                x = x + self.width
                continue
            width, bytes_per_line = ustruct.unpack('HH', data[:4])
            # print('character [%d]: width = %d, bytes_per_line = %d' % (ord(c)
            # , width, bytes_per_line))
            for h in range(0, self.f.height):
                w = 0
                i = 0
                while w < width:
                    mask = data[4 + h * bytes_per_line + i]
                    if (width - w) >= 8:
                        n = 8
                    else:
                        n = width - w
                    py = y + h
                    page = py >> 3
                    bit = 0x80 >> (py % 8)
                    for p in range(0, n):
                        px = x + w + p
                        c = 0
                        if (mask & 0x80) != 0:
                            if mode == TextMode.normal or \
                               mode == TextMode.trans:
                                c = 1
                            if mode == TextMode.rev:
                                c = 0
                            if mode == TextMode.xor:
                                c = self.buffer[page * 128 + px] & bit
                                if c != 0:
                                    c = 0
                                else:
                                    c = 1
                                # print("px = %d, py = %d, c = %d" % (px, py, c))
                            super().pixel(px, py, c)
                        else:
                            if mode == TextMode.normal:
                                c = 0
                                super().pixel(px, py, c)
                            if mode == TextMode.rev:
                                c = 1
                                super().pixel(px, py, c)
                        mask = mask << 1
                    w = w + 8
                    i = i + 1
            x = x + width + 1

class Mpu6050_acce_fs():
    ACCE_FS_2G  = 0x00     #Accelerometer full scale range is +/- 2g 
    ACCE_FS_4G  = 0x01     #Accelerometer full scale range is +/- 4g 
    ACCE_FS_8G  = 0x02     #Accelerometer full scale range is +/- 8g 
    ACCE_FS_16G = 0x03     # Accelerometer full scale range is +/- 16g 
class Mpu6050_gyro_fs():
    GYRO_FS_250DPS  = 0     # Gyroscope full scale range is +/- 250 degree per sencond 
    GYRO_FS_500DPS  = 1     # Gyroscope full scale range is +/- 500 degree per sencond 
    GYRO_FS_1000DPS = 2     # Gyroscope full scale range is +/- 1000 degree per sencond 
    GYRO_FS_2000DPS = 3     # Gyroscope full scale range is +/- 2000 degree per sencond 
class Bit():
    BIT0 = 0x1
    BIT1 = 0x2
    BIT2 = 0x4
    BIT3 = 0x8
    BIT4 = 0x10
    BIT5 = 0x20
    BIT6 = 0x40
    BIT7 = 0x80

class Mpu6050():
    """  """

    def __init__(self, acce_fs = Mpu6050_acce_fs.ACCE_FS_2G, gyro_fs  = Mpu6050_gyro_fs.GYRO_FS_500DPS):
        self.addr = 104
        self.acce_fs = acce_fs
        self.gyro_fs = gyro_fs
        self.acce_sensitivity = 16384  >> self.acce_fs
        self.gyro_sensitivity = (1310 >> self.gyro_fs)/10
        # print("acce_sensitivity: %d" % self.acce_sensitivity)
        # print("gyro_sensitivity: %.2f" % self.gyro_sensitivity)
        self.wake_up()
        self.set_acce_fs(self.acce_fs)
        self.set_gyro_fs(self.gyro_fs)
        self.set_aux_i2c_mode(0)

    def wake_up(self):
        i2c.writeto(self.addr, b'\x6B')  
        pwr_mgmt = i2c.readfrom(self.addr, 1)
        _pwr_mgmt = pwr_mgmt[0] & (~Bit.BIT6) #0xBF
        i2c.writeto(self.addr, bytearray([0x6B, _pwr_mgmt]))
        i2c.writeto(self.addr, b'\x6B')  
        pwr_mgmt = i2c.readfrom(self.addr, 1)

    def set_aux_i2c_mode(self, mode):
        i2c.writeto(self.addr, b'\x6A')  
        use_ctrl = i2c.readfrom(self.addr, 1)
        _use_ctrl =  use_ctrl[0]
        i2c.writeto(self.addr, b'\x37') 
        bypass = i2c.readfrom(self.addr, 1)
        _bypass =  bypass[0]
        if(mode == 1):
            _bypass &= 0xfd
            _use_ctrl |=  0x20
        elif (mode == 0):
            _bypass |= 0x02
            _use_ctrl &= 0xdf 
        i2c.writeto(self.addr, bytearray([0x6A, _use_ctrl]))
        i2c.writeto(self.addr, bytearray([0x37, _bypass]))

    def get_device_id(self):
        i2c.writeto(self.addr, b'\x75') 
        id = i2c.readfrom(self.addr, 1)
        return id[0]

    def set_acce_fs(self, acce_fs):
        i2c.writeto(self.addr, b'\x1C')  
        afs = i2c.readfrom(self.addr, 1)
        _afs = afs[0] & 0xE7
        _afs |= acce_fs << 3
        i2c.writeto(self.addr, bytearray([0x1C, _afs]))
        # i2c.writeto(self.addr, b'\x1C')  
        # afs = i2c.readfrom(self.addr, 1)
        # print("afs: %d" % afs[0])

    def set_gyro_fs(self, gyro_fs):
        i2c.writeto(self.addr, b'\x1B')  
        gfs = i2c.readfrom(self.addr, 1)
        _gfs = gfs[0] & 0xE7
        _gfs |= gyro_fs << 3
        i2c.writeto(self.addr, bytearray([0x1B, _gfs]))
        # i2c.writeto(self.addr, b'\x1B')  
        # gfs = i2c.readfrom(self.addr, 1)
        # print("gfs: %d" %  gfs[0])

    def get_raw_acce(self):
        i2c.writeto(self.addr, b'\x3B') 
        acce = i2c.readfrom(self.addr, 6)
        _acce = ustruct.unpack('!3h', acce)
        return _acce

    def get_raw_gyro(self):
        i2c.writeto(self.addr, b'\x43') 
        gyro = i2c.readfrom(self.addr, 6)
        _gyro = ustruct.unpack('!3h', gyro)
        return _gyro 

    def get_temp(self):
        i2c.writeto(self.addr, b'\x41') 
        temp = i2c.readfrom(self.addr, 2)
        _temp = ustruct.unpack('!h', temp)[0]
        return _temp /340 + 36.53 

class Accelerometer():
    """  """

    def __init__(self):
        self.mpu = mpu6050

    def get_x(self):
        acce_x = self.mpu.get_raw_acce()[0]
        return acce_x/self.mpu.acce_sensitivity

    def get_y(self):
        acce_y = self.mpu.get_raw_acce()[1]
        return acce_y/self.mpu.acce_sensitivity

    def get_z(self):
        acce_z = self.mpu.get_raw_acce()[2]
        return acce_z/self.mpu.acce_sensitivity

class Gyro():
    """  """

    def __init__(self):
        self.mpu = mpu6050

    def get_x(self):
        gyro_x = self.mpu.get_raw_gyro()[0]
        return gyro_x/self.mpu.gyro_sensitivity

    def get_y(self):
        gyro_y = self.mpu.get_raw_gyro()[1]
        return gyro_y/self.mpu.gyro_sensitivity

    def get_z(self):
        gyro_z = self.mpu.get_raw_gyro()[2]
        return gyro_z/self.mpu.gyro_sensitivity

class Hmc5583l_samples():
    HMC5883L_SAMPLES_8  = 0b11
    HMC5883L_SAMPLES_4  = 0b10
    HMC5883L_SAMPLES_2  = 0b01
    HMC5883L_SAMPLES_1  = 0b00
class Hmc5583l_data_rate():
    HMC5883L_DATARATE_75HZ     = 0b110
    HMC5883L_DATARATE_30HZ     = 0b101
    HMC5883L_DATARATE_15HZ     = 0b100
    HMC5883L_DATARATE_7_5HZ    = 0b011
    HMC5883L_DATARATE_3HZ      = 0b010
    HMC5883L_DATARATE_1_5HZ    = 0b001
    HMC5883L_DATARATE_0_75_HZ  = 0b000
class Hmc5583l_range():
    HMC5883L_RANGE_8_1GA     = 0b111
    HMC5883L_RANGE_5_6GA     = 0b110
    HMC5883L_RANGE_4_7GA     = 0b101
    HMC5883L_RANGE_4GA       = 0b100
    HMC5883L_RANGE_2_5GA     = 0b011
    HMC5883L_RANGE_1_9GA     = 0b010
    HMC5883L_RANGE_1_3GA     = 0b001
    HMC5883L_RANGE_0_88GA    = 0b000
class Hmc5583l_mode():
    HMC5883L_IDLE          = 0b10
    HMC5883L_SINGLE        = 0b01
    HMC5883L_CONTINOUS     = 0b00
class Hmc5583l():
    """  """

    def __init__(self, range = Hmc5583l_range.HMC5883L_RANGE_1_3GA, mode = Hmc5583l_mode.HMC5883L_CONTINOUS, \
                    data_rate = Hmc5583l_data_rate.HMC5883L_DATARATE_30HZ, samples = Hmc5583l_samples.HMC5883L_SAMPLES_8):
        self.addr = 30
        self.range = range
        self.mode = mode
        self.data_rate = data_rate
        self.samples = samples
        self.mgPerDigit = 0.92
        self.x_off = 0
        self.y_off = 0
        self.set_range(self.range)
        self.set_mode(self.mode)
        self.set_data_rate(self.data_rate)
        self.set_samples(self.samples)
        # test
        # i2c.writeto(self.addr, b'\x00')  # HMC5883L_REG_CONFIG_A
        # reg = i2c.readfrom(self.addr, 3)
        # print("CFG_A reg: %d, CFG_B reg: %d, MODE reg: %d, mgPerDigit: %.2f," % (reg[0], reg[1], reg[2], self.mgPerDigit))

    def set_offset(self, x_off, y_off):
        self.x_off = x_off
        self.y_off = y_off

    def set_range(self, range):
        if range == 0: 
	        self.mgPerDigit = 0.073
        elif range == 1:
            self.mgPerDigit = 0.92
        elif range == 2:
            self.mgPerDigit = 1.22
        elif range == 3:
            self.mgPerDigit = 1.52
        elif range == 4:
            self.mgPerDigit = 2.27
        elif range == 5:
            self.mgPerDigit = 2.56
        elif range == 6:
            self.mgPerDigit = 3.03
        elif range == 7:
            self.mgPerDigit = 4.35
        i2c.writeto(self.addr, bytearray([0x01, (range << 5)]))

    def set_mode(self, mode):
        i2c.writeto(self.addr, b'\x02')  # HMC5883L_REG_CONFIG_A
        reg = i2c.readfrom(self.addr, 1)
        _reg = reg[0] & 0xfc
        _reg |= mode
        i2c.writeto(self.addr, bytearray([0x02, _reg]))  

    def set_data_rate(self, data_rate):
        i2c.writeto(self.addr, b'\x00')  # HMC5883L_REG_CONFIG_A
        reg = i2c.readfrom(self.addr, 1)
        _reg = reg[0] & 0xe3
        _reg |= ((data_rate << 2))
        i2c.writeto(self.addr, bytearray([0x00, _reg]))   

    def set_samples(self, samples):
        i2c.writeto(self.addr, b'\x00')  # HMC5883L_REG_CONFIG_A
        reg = i2c.readfrom(self.addr, 1)
        _reg = reg[0] & 0x9f
        _reg |= ((samples << 5))
        i2c.writeto(self.addr, bytearray([0x00, _reg]))     

    def get_raw(self):
        i2c.writeto(self.addr, b'\x03') 
        raw = i2c.readfrom(self.addr, 6)
        _raw = ustruct.unpack('!3h', raw)
        return _raw

    def get_x(self):
        x = (self.get_raw()[0] - self.x_off) * self.mgPerDigit
        return x

    def get_y(self):
        y = (self.get_raw()[1]  - self.y_off) * self.mgPerDigit
        return y

    def get_z(self):
        z = self.get_raw()[2] * self.mgPerDigit
        return z

    def get_angle(self, longitude):
        heading = math.atan2(self.get_y(), self.get_x())
        declinationAngle = longitude / (180 / math.pi)
        heading += declinationAngle
        if (heading < 0):
            heading += 2 * math.pi
        if (heading > 2 * math.pi):
            heading -= 2 * math.pi
        headingDegrees = heading * 180/math.pi
        return headingDegrees

class BME280(object):
    def __init__(self):
        self.addr = 119
        # The “ctrl_hum” register sets the humidity data acquisition options of the device
        # 0x01 = [2:0]oversampling ×1
        i2c.writeto(self.addr, b'\xF2\x01')
        # The “ctrl_meas” register sets the pressure and temperature data acquisition options of the device.
        # The register needs to be written after changing “ctrl_hum” for the changes to become effective.
        # 0x27 = [7:5]Pressure oversampling ×1 | [4:2]Temperature oversampling ×4 | [1:0]Normal mode
        i2c.writeto(self.addr, b'\xF4\x27')
        # The “config” register sets the rate, filter and interface options of the device. Writes to the “config”
        # register in normal mode may be ignored. In sleep mode writes are not ignored.
        i2c.writeto(self.addr, b'\xF5\x00')

        i2c.writeto(self.addr, b'\x88', False)
        bytes = i2c.readfrom(self.addr, 6)
        self.dig_T = ustruct.unpack('Hhh', bytes)

        i2c.writeto(self.addr, b'\x8E', False)
        bytes = i2c.readfrom(self.addr, 18)
        self.dig_P = ustruct.unpack('Hhhhhhhhh', bytes)

        i2c.writeto(self.addr, b'\xA1', False)
        self.dig_H = array.array('h', [0, 0, 0, 0, 0, 0])
        self.dig_H[0] = i2c.readfrom(self.addr, 1)[0]
        i2c.writeto(self.addr, b'\xE1', False)
        buff = i2c.readfrom(self.addr, 7)
        self.dig_H[1] = ustruct.unpack('h', buff[0:2])[0]
        self.dig_H[2] = buff[2]
        self.dig_H[3] = (buff[3] << 4) | (buff[4] & 0x0F)
        self.dig_H[4] = (buff[5] << 4) | (buff[4] >> 4 & 0x0F)
        self.dig_H[5] = buff[6]

    def temperature(self):
        retry = 0
        if (retry < 5):
            try:
                i2c.writeto(self.addr, b'\xFA', False)
                buff = i2c.readfrom(self.addr, 3)
                T = (((buff[0] << 8) | buff[1]) << 4) | (buff[2] >> 4 & 0x0F)
                c1 = (T / 16384.0 - self.dig_T[0] / 1024.0) * self.dig_T[1]
                c2 = ((T / 131072.0 - self.dig_T[0] / 8192.0) * (T / 131072.0 - self.dig_T[0] / 8192.0)) * self.dig_T[2]
                self.tFine = c1 + c2
                return self.tFine / 5120.0
            except:
                retry = retry + 1
        else:
            raise Exception("i2c read/write error!")

    def pressure(self):
        retry = 0
        if (retry < 5):
            try:
                i2c.writeto(self.addr, b'\xF7', False)
                buff = i2c.readfrom(self.addr, 3)
                P = (((buff[0] << 8) | buff[1]) << 4) | (buff[2] >> 4 & 0x0F)
                c1 = self.tFine / 2.0 - 64000.0
                c2 = c1 * c1 * self.dig_P[5] / 32768.0
                c2 = c2 + c1 * self.dig_P[4] * 2.0
                c2 = c2 / 4.0 + self.dig_P[3] * 65536.0
                c1 = (self.dig_P[2] * c1 * c1 / 524288.0 + self.dig_P[1] * c1) / 524288.0
                c1 = (1.0 + c1 / 32768.0) * self.dig_P[0]
                if c1 == 0.0:
                    return 0
                p = 1048576.0 - P
                p = (p - c2 / 4096.0) * 6250.0 / c1
                c1 = self.dig_P[8] * p * p / 2147483648.0
                c2 = p * self.dig_P[7] / 32768.0
                p = p + (c1 + c2 + self.dig_P[6]) / 16.0
                return p
            except:
                retry = retry + 1
        else:
            raise Exception("i2c read/write error!")

    def humidity(self):
        retry = 0
        if (retry < 5):
            try:
                self.temperature()
                i2c.writeto(self.addr, b'\xFD', False)
                buff = i2c.readfrom(self.addr, 2)
                H = buff[0] << 8 | buff[1]
                h = self.tFine - 76800.0
                h = (H - (self.dig_H[3] * 64.0 + self.dig_H[4] / 16384.0 * h)) * \
                    (self.dig_H[1] / 65536.0 * (1.0 + self.dig_H[5] / 67108864.0 * h * \
                    (1.0 + self.dig_H[2] / 67108864.0 * h)))
                h = h * (1.0 - self.dig_H[0] * h / 524288.0)
                if h > 100.0:
                    return 100.0
                elif h < 0.0:
                    return 0.0
                else:
                    return h
            except:
                retry = retry + 1
        else:
            raise Exception("i2c read/write error!")

class BS8112A():
    """  """

    def __init__(self):
        self.addr = 80
        # config
        self.config = [0xB0, 0x00, 0x00, 0x83, 0xf3, 0x98, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x00] 
        checksum = 0
        for i in range(1, 19):
            checksum += self.config[i]
        checksum &= 0xff
        self.config[18] = checksum
        # print(self.config[18])
        i2c.writeto(self.addr, bytearray(self.config), True) 
        # i2c.writeto(self.addr, b'\xB0', False) 
        # time.sleep_ms(10)
        # print(i2c.readfrom(self.addr, 17, True))

    # key map:
    # value       bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0 
    # bs8112a key Key8 Key7 Key6 Key5 Key4 Key3 Key2 Key1
    # mpython key       N    O    H    T    Y    P
    def key_value(self):
        i2c.writeto(self.addr, b'\x08', False)  
        time.sleep_ms(10)
        value = i2c.readfrom(self.addr, 1, True)[0]
        time.sleep_ms(10)
        return value

class Codec_mode():
    ES_MODULE_ADC_DAC  = 0x00      
    ES_MODULE_DAC  = 0x01    
    ES_MODULE_ADC  = 0x02     

class Es8388():
    """  """

    def __init__(self, adc_volume = 0, dac_volume  = 0, volume = 65):
        self.addr = 16
        self.adc_volume = adc_volume
        self.dac_volume = dac_volume
        self.volume = volume
        self.set_voice_mute(1)
        # i2c.writeto(self.addr, bytearray([0x19, 0x04])) # ES8388_DACCONTROL3 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
        #  Chip Control and Power Management 
        i2c.writeto(self.addr, bytearray([0x01, 0x50])) # ES8388_CONTROL2 0x40? 
        i2c.writeto(self.addr, bytearray([0x02, 0x00])) # ES8388_CHIPPOWER normal all and power up all
        i2c.writeto(self.addr, bytearray([0x08, 0x00])) # ES8388_MASTERMODE CODEC IN I2S SLAVE MODE 0x00: slave
        # dac setup
        i2c.writeto(self.addr, bytearray([0x04, 0xC0])) # ES8388_DACPOWER . disable DAC and disable Lout/Rout/1/2
        i2c.writeto(self.addr, bytearray([0x00, 0x12])) # ES8388_CONTROL1. Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
        i2c.writeto(self.addr, bytearray([0x17, 0x18])) # ES8388_DACCONTROL1 1a 0x18:16bit iis , 0x00:24
        i2c.writeto(self.addr, bytearray([0x18, 0x02])) # ES8388_DACCONTROL2 DACFsMode,SINGLE SPEED; DACFsRatio,256
        i2c.writeto(self.addr, bytearray([0x26, 0x00])) # ES8388_DACCONTROL16 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
        i2c.writeto(self.addr, bytearray([0x27, 0x90])) # ES8388_DACCONTROL17 only left DAC to left mixer enable 0db
        i2c.writeto(self.addr, bytearray([0x2a, 0x90])) # ES8388_DACCONTROL20 only right DAC to right mixer enable 0db
        i2c.writeto(self.addr, bytearray([0x2b, 0x80])) # ES8388_DACCONTROL21 set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
        i2c.writeto(self.addr, bytearray([0x2d, 0x00])) # ES8388_DACCONTROL23 vroi=0
        self.set_adc_dac_volume(Codec_mode.ES_MODULE_DAC, self.dac_volume, 0) #0db
        i2c.writeto(self.addr, bytearray([0x04, 0x3c])) # ES8388_DACPOWER 0x3c Enable DAC and Enable Lout/Rout/1/2
        # adc setup
        i2c.writeto(self.addr, bytearray([0x03, 0xff])) # ES8388_ADCPOWER
        i2c.writeto(self.addr, bytearray([0x09, 0xbb])) # ES8388_ADCCONTROL1 MIC Left and Right channel PGA gain
        i2c.writeto(self.addr, bytearray([0x0a, 0x00])) # ES8388_ADCCONTROL2 0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
        i2c.writeto(self.addr, bytearray([0x0b, 0x02])) # ES8388_ADCCONTROL3 clock input
        i2c.writeto(self.addr, bytearray([0x0c, 0x0c])) # ES8388_ADCCONTROL4 Left/Right data, Left/Right justified mode, Bits length 16bit, I2S format 0x0c?
        i2c.writeto(self.addr, bytearray([0x0d, 0x02])) # ES8388_ADCCONTROL5 ADCFsMode,singel SPEED,RATIO=256
        # ALC for Microphone
        self.set_adc_dac_volume(Codec_mode.ES_MODULE_ADC, self.adc_volume, 0) #0db
        i2c.writeto(self.addr, bytearray([0x03, 0x09])) # ES8388_ADCPOWER Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode
        # set volume
        self.set_volume(self.volume)
        self.set_voice_mute(0)
        # test 
        # for i in range(0, 52):
        #     i2c.writeto(self.addr, bytearray([i]))
        #     print("%d: %d" % (i, i2c.readfrom(self.addr, 1)[0]))

    def deinit(self):
        i2c.writeto(self.addr, bytearray([0x02, 0xff])) # ES8388_CHIPPOWER reset and stop es838

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
        if (mode == Codec_mode.ES_MODULE_ADC or mode == Codec_mode.ES_MODULE_ADC_DAC) :
            i2c.writeto(self.addr, bytearray([0x10, _volume])) # ES8388_ADCCONTROL8
            i2c.writeto(self.addr, bytearray([0x11, _volume])) # ES8388_ADCCONTROL9
        if (mode == Codec_mode.ES_MODULE_DAC or mode == Codec_mode.ES_MODULE_ADC_DAC) :
            i2c.writeto(self.addr, bytearray([0x1b, _volume])) # ES8388_DACCONTROL5
            i2c.writeto(self.addr, bytearray([0x1a, _volume])) # ES8388_DACCONTROL4

    def set_volume(self, volume):
        self.volume = volume
        if (self.volume < 0):
            self.volume = 0
        elif (self.volume > 100):
            self.volume = 100
        i2c.writeto(self.addr, bytearray([0x2e, self.volume//3])) # ES8388_DACCONTROL24
        i2c.writeto(self.addr, bytearray([0x2f, self.volume//3])) # ES8388_DACCONTROL25
        i2c.writeto(self.addr, bytearray([0x30, 0])) # ES8388_DACCONTROL26
        i2c.writeto(self.addr, bytearray([0x31, 0])) # ES8388_DACCONTROL27
        # print("volume L: %d" % (self.volume//3))

    def get_volume(self):
        return self.volume

    def set_voice_mute(self, mute):
        i2c.writeto(self.addr, b'\x19')  
        dac_ctr3 = i2c.readfrom(self.addr, 1)[0]
        if(mute):
            dac_ctr3 |= 0x04
        else:
            dac_ctr3 &= 0xFB
        i2c.writeto(self.addr, bytearray([0x19, dac_ctr3]))

class PinMode(object):
    IN = 1
    OUT = 2
    PWM = 3
    ANALOG = 4
    OUT_DRAIN = 5

pins_remap_esp32 = (33, 32, 35, 34, 39, 0, 16, 17, 26, 25, 36, 2, -1, 18, 19, 21, 5, -1, -1, 22, 23, -1, -1, 27, 14, 12,
                    13, 15, 4)

class MPythonPin():
    def __init__(self, pin, mode=PinMode.IN, pull=None):
        if mode not in [PinMode.IN, PinMode.OUT, PinMode.PWM, PinMode.ANALOG, PinMode.OUT_DRAIN]:
            raise TypeError("mode must be 'IN, OUT, PWM, ANALOG,OUT_DRAIN'")
        if pin == 4:
            raise TypeError("P4 is used for light sensor")
        if pin == 10:
            raise TypeError("P10 is used for sound sensor")
        try:
            self.id = pins_remap_esp32[pin]
        except IndexError:
            raise IndexError("Out of Pin range")
        if mode == PinMode.IN:
            if pin in [3]:
                raise TypeError('IN not supported on P%d' % pin)
            self.Pin = Pin(self.id, Pin.IN, pull)
        if mode == PinMode.OUT:
            if pin in [2, 3]:
                raise TypeError('OUT not supported on P%d' % pin)
            self.Pin = Pin(self.id, Pin.OUT, pull)
        if mode == PinMode.OUT_DRAIN:
            if pin in [2, 3]:
                raise TypeError('OUT_DRAIN not supported on P%d' % pin)
            self.Pin = Pin(self.id, Pin.OPEN_DRAIN, pull)
        if mode == PinMode.PWM:
            if pin not in [0, 1, 5, 6, 7, 8, 9, 11, 13, 14, 15, 16, 19, 20, 23, 24, 25, 26, 27, 28]:
                raise TypeError('PWM not supported on P%d' % pin)
            self.pwm = PWM(Pin(self.id), duty=0)
        if mode == PinMode.ANALOG:
            if pin not in [0, 1, 2, 3, 4, 10]:
                raise TypeError('ANALOG not supported on P%d' % pin)
            self.adc = ADC(Pin(self.id))
            self.adc.atten(ADC.ATTN_11DB)
        self.mode = mode

    def irq(self, handler=None, trigger=Pin.IRQ_RISING):
        if not self.mode == PinMode.IN:
            raise TypeError('the pin is not in IN mode')
        return self.Pin.irq(handler, trigger)

    def read_digital(self):
        if not self.mode == PinMode.IN:
            raise TypeError('the pin is not in IN mode')
        return self.Pin.value()

    def write_digital(self, value):
        if self.mode not in [PinMode.OUT, PinMode.OUT_DRAIN]:
            raise TypeError('the pin is not in OUT or OUT_DRAIN mode')
        self.Pin.value(value)

    def read_analog(self):
        if not self.mode == PinMode.ANALOG:
            raise TypeError('the pin is not in ANALOG mode')
        # calibration esp32 ADC 
        calibration_val = 0
        val = int(sum([self.adc.read() for i in range(50)]) / 50)
        if 0 < val <= 2855:
            calibration_val = 1.023 * val + 183.6
        if 2855 < val <= 3720:
            calibration_val = 0.9769 * val + 181
        if 3720 < val <= 4095:
            calibration_val = 4095 - (4095 - val) * 0.2
        return calibration_val

    def write_analog(self, duty, freq=1000):
        if not self.mode == PinMode.PWM:
            raise TypeError('the pin is not in PWM mode')
        self.pwm.freq(freq)
        self.pwm.duty(duty)

class wifi:
    def __init__(self):
        self.sta = network.WLAN(network.STA_IF)
        self.ap = network.WLAN(network.AP_IF)

    def connectWiFi(self, ssid, passwd, timeout=10):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(True)
        list = self.sta.scan()
        for i, wifi_info in enumerate(list):
            try:
                if wifi_info[0].decode() == ssid:
                    self.sta.connect(ssid, passwd)
                    wifi_dbm = wifi_info[3]
                    break
            except UnicodeError:
                self.sta.connect(ssid, passwd)
                wifi_dbm = '?'
                break
            if i == len(list) - 1:
                raise OSError("SSID invalid / failed to scan this wifi")
        start = time.time()
        print("Connection WiFi", end="")
        while (self.sta.ifconfig()[0] == '0.0.0.0'):
            if time.ticks_diff(time.time(), start) > timeout:
                print("")
                raise OSError("Timeout!,check your wifi password and keep your network unblocked")
            print(".", end="")
            time.sleep_ms(500)
        print("")
        print('WiFi(%s,%sdBm) Connection Successful, Config:%s' % (ssid, str(wifi_dbm), str(self.sta.ifconfig())))

    def disconnectWiFi(self):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(False)
        print('disconnect WiFi...')

    def enable_APWiFi(self, essid, channel=10):
        self.ap.active(True)
        self.ap.config(essid=essid, channel=channel)

    def disable_APWiFi(self):
        self.ap.disconnect()
        self.ap.active(False)
        print('disable AP WiFi...')

# display
if 60 in i2c.scan():
    oled = OLED()
    display = oled

# 9 axis
mpu6050 = Mpu6050()
accelerometer = Accelerometer()
gyro = Gyro()
compass = Hmc5583l()

# bm280
if 119 in i2c.scan():
    bme280 = BME280()

# 3 rgb leds
rgb = NeoPixel(Pin(25, Pin.OUT), 25, 3, 1)
rgb.write()

# light sensor
light = ADC(Pin(39))
light.atten(light.ATTN_11DB)

# sound sensor
sound = ADC(Pin(36))
sound.atten(sound.ATTN_11DB)

# slide resistance
slider_res =  ADC(Pin(34))
slider_res.atten(slider_res.ATTN_11DB)

# buttons
button_a = Pin(0, Pin.IN, Pin.PULL_UP)
button_b = Pin(2, Pin.IN, Pin.PULL_UP)

# touchpad
touch_pad = BS8112A()

# human infrared
pir = Pin(21, mode = Pin.IN, pull = None)

# codec es8388
es8388 = Es8388()

from gui import *

def numberMap(inputNum, bMin, bMax, cMin, cMax):
    outputNum = 0
    outputNum = ((cMax - cMin) / (bMax - bMin)) * (inputNum - bMin) + cMin
    return outputNum