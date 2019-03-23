"""
Author:     Patrick Tafoya
Purpose:    Test the capability of the TCA9548A to process
            i2c signals from different logic voltage levels.
            
            TCA is wired to i2c-1 on the Rpi
            MLX90614 is wired to channel 0 on TCA at 3.3V logic
            BNO055 is wired to channel 2 on TCA at 3.3V logic
            ADS1115 is wired to channel 7 on TCA at 5V logic
                Potentiometer is wired to A0 on ADS
"""
import smbus
from time import sleep
from mlx90614 import MLX90614
import Adafruit_ADS1x15
from Adafruit_BNO055 import BNO055
#initalize SMBus on i2c-1
i2c = smbus.SMBus(1)
#MUX address
MUX = 0x70
#close all MUX channels
i2c.write_byte(MUX, 0)
#initialize IR thermometer i2c addr 0x5B on channel 0
i2c.write_byte(MUX, (1<<0))
IR = MLX90614(0x5B)
#initialize IMU i2c addr 0x28 on channel 2
i2c.write_byte(MUX, (1<<2))
IMU = BNO055(0x28)
IMU.begin()
#initialize ADC i2c addr 0x48 on channel 7
i2c.write_byte(MUX, (1<<7))
ADC = Adafruit_ADS1x15.ADS1115(address=0x48)
#conversion for ADC read
CONVERT = (6.144/32767)

def tempRead():
    #select channel 0
    i2c.write_byte(MUX, (1<<0))
    data = IR.get_obj_temp_C()
    return data

def orientRead():
    #select channel 2
    i2c.write_byte(MUX, (1<<2))
    data = IMU.read_euler()
    return(data)

def voltRead():
    #select channel 7
    i2c.write_byte(MUX, (1<<7))
    data = ADC.read_adc(0, (2/3)) * CONVERT
    return data


if __name__ == '__main__':
    while True:
        try:
            temp = tempRead()
        except IOError:
            temp = 0
        try:
            orient = orientRead()
        except IOError:
            orient = [0, 0, 0]
        try:
            volts = voltRead()
        except IOError:
            volts = 0
        print("%.2f C\t" % temp, "X: %.2f\t" % orient[0], "Y: %.2f\t" % orient[1], "Z: %.2f\t" % orient[2], "%.2f V" % volts)
        sleep(0.1)
