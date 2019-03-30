import smbus
from time import sleep
import Adafruit_ADS1x15

#   2.97V = max output = 250psi
#   2.97V / 4.096 V = 0.7251
#   250psi / 0.7251 = 344.781 psi
#   344.781 psi / 4.096 V = 59.761 psi / V

CONVERT = (4.096/32767)
V2PSI = 59.761

GAIN = 1
ADDR = 0x48

ADC = Adafruit_ADS1x15.ADS1115(address=ADDR, busnum=1)
bus = smbus.SMBus(1)

try:
    bus.write_quick(ADDR)
except:
    print("ERROR")

if __name__ == '__main__':
    while True:
        try:
            pressure = ((ADC.read_adc(0, GAIN) * CONVERT) - 0.29)
        except IOError:
            pressure = 0
        print("%.2f psi" % pressure)
        sleep(0.1)
