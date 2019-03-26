import smbus
from time import sleep
from HyperlynxBMP280 import BMP280

bmp = BMP280()

if __name__ == "__main__":
    temp = bmp.read_temperature()
    press = bmp.read_pressure()
    print("%.2f C\t" % temp, "%.2f psi" % press)
    sleep(0.1)
