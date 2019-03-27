import smbus
from time import sleep
import Lidar

bus = smbus.SMBus(1)

lid = Lidar.Lidar_Lite()

if __name__ == '__main__':
	while True:
		try:
			dist = lid.getDistance()
		except IOError:
			dist = 0
		print("%.2f cm" % dist)
		sleep(0.1)
