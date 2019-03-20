"""
ADDR => GND => I2C ADDRESS = 0x48

From datasheet ADS1115 returns values from -32768 to 32767
Gain set as 1 measures up to 4.096 V
32767 / 4.096V = 7999.75
Therefore, we must divide reading by 7999.75 for voltage result
"""
CONVERT = 7999.75
ADDR = 0x48
import smbus
from time import sleep
import Adafruit_ADS1x15
#Object for use
ADC = Adafruit_ADS1x15.ADS1115(address=ADDR, busnum=1)
bus = smbus.SMBus(1)

#Quick check for I2C connection
try:
    bus.write_quick(ADDR)
except:
    print("ERROR")
#Start continuous conversions on A0   
#ADC.start_adc(0, gain=1)

if __name__ == '__main__':
    while True:
        try:
            #READ SINGLE VOLTAGE ON A0
            volt = (ADC.read_adc(0, 1) / CONVERT)
        except IOError:
            volt = 0
        try:
            #READ SINGLE CURRENT ON A1
            amps = (ADC.read_adc(1, 1) / CONVERT)
        except IOError:
            amps = 0
        
        print("%.2f V\t" % volt, "%.2f A" % amps)
        sleep(0.1)
