#ADDR --> GND   0x48
import smbus
from time import sleep
from Adafruit_Python_ADS1x15.Adafruit_ADS1x15.ADS1x15 import ADS1115 as ADC

bus = smbus.SMBus(1)
ADS1x15_CONFIG_MODE_CONTINUOUS = 0x0000

try:
    bus.write_quick(0x48)
except:
    print("ERROR")
    
ADC.start_adc(ADC, 0, 1, None)

if __name__ == '__main__':
    while True:
        data = ADC._read(ADC, 0, 1, 0, 0)
        print(data)
        sleep(0.5)
  
