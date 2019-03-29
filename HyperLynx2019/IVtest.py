"""
ADDR => GND => I2C ADDRESS = 0x48

From datasheet ADS1115 returns values from -32768 to 32767
Gain set as 1 measures up to 4.096 V
32767 / 4.096V = 7999.75
Therefore, we must divide reading by 7999.75 for voltage result

attopilot V ratio = 1 : 4.13
attopilot 
"""
CONVERT = (4.096/32767)
vRatio = 4.13
ADDR = 0x48
GAIN = (1)
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
            volt = (ADC.read_adc(0, GAIN) * CONVERT) * vRatio
        except IOError:
            volt = 0
        try:
            #READ SINGLE CURRENT ON A1
            amps = (ADC.read_adc(1, GAIN) * CONVERT)
        except IOError:
            amps = 0
        amp = volt/219.2
        
        print("%.6f V\t" % volt, "%.6f\t" % amps, "%.6f A" % amp)
        sleep(0.1)
