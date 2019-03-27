"""
Author:		Patrick Tafoya
Purpose:	Test full ECS

			bus 0:
				LIDAR lite v3	5v
			bus 1:
				BNO055 at 0x28
				BNO055 at 0x29
				BMP280 at 0x77
			bus 2:
				BME280 at 0x76
			bus 3:
				MLX90614 at 0x5B
				BME280 at 0x77
				ADS1115 at 0x48	gain = 1 (4.096V MAX)
"""
import smbus
from time import sleep
from mlx90614 import MLX90614
from Adafruit_BNO055 import BNO055
import Adafruit_ADS1x15
from HyperlynxBMP280 import BMP280
from Adafruit_BME280 import BME280
import Lidar
import RPi.GPIO

class HyperlynxECS():
	def __init__(self, bus_num=1):
		self.MUX_ADDR = 0x70
		self.IR_ADDR = 0x5B
		self.IMU_ADDR1 = 0x29
		self.IMU_ADDR2 = 0x28
		self.BMP_ADDR = 0x77
		self.BME_ADDR1 = 0x77
		self.BME_ADDR2 = 0x76
		self.LID_ADDR = 0x62
		self.ADC_ADDR = 0x48
		self.METER2G = (3.28084/32.2)
		self.VOLT = 0
		self.AMP = 1
		self.ADC_GAIN = 1
		self.ADC_CONVERT = (4.096/32767.0)
		self.IO = RPi.GPIO
		self.NOsolPIN = 17
		self.NCsol1PIN = 27
		self.NCsol2PIN = 22
		self.bus = smbus.SMBus(bus_num)
		self.closeAllBus()
		self.IO.setmode(self.IO.BCM)
		self.IO.setwarnings(False)
		self.currentBus = 0
		
	def initializeSensors(self): 
		self.openBus(0)
		self.Lidar = Lidar.Lidar_Lite()
		try:
			self.bus.write_quick(self.LID_ADDR)
			print("Lidar Ready")
		except IOError:
			print("Connection error with Lidar")
		self.openBus(1)
		self.IMU1 = BNO055(address=self.IMU_ADDR1)
		if(self.IMU1.begin()):
			print("IMU1 Ready")
		else:
			print("Connection Error with IMU1")
		self.IMU2 = BNO055(address=self.IMU_ADDR2)
		if(self.IMU2.begin()):
			print("IMU2 Ready")
		else:
			print("Connection Error with IMU2")
		self.BMP = BMP280(address=self.BMP_ADDR)
		try:
			self.bus.write_quick(self.BMP_ADDR)
			print("BMP Ready")
		except IOError:
			print("Connection Error with BMP")
		self.openBus(2)
		self.BME1 = BME280(address=self.BME_ADDR2)
		try:
			self.bus.write_quick(self.BME_ADDR2)
			print("BME2 Ready")
		except IOError:
			print("Connection Error with BME2")
		self.openBus(3)
		self.Therm = MLX90614(address=self.IR_ADDR)
		try:
			self.bus.write_quick(self.IR_ADDR)
			print("Therm Ready")
		except IOError:
			print("Connection Error with Therm")
		self.ADC = Adafruit_ADS1x15.ADS1115(address=self.ADC_ADDR, busnum=1)
		try:
			self.bus.write_quick(self.ADC_ADDR)
			print("ADC Ready")
		except IOError:
			print("Connection Error with ADC")
		self.BME2 = BME280(address=self.BME_ADDR1)
		try:
			self.bus.write_quick(self.BME_ADDR1)
			print("BME1 Ready")
		except IOError:
			print("Connection Error with BME1")
		
		

	def closeAllBus(self):
		self.bus.write_byte(self.MUX_ADDR, 0)
		self.currentBus = 0
		
	def openBus(self, bus_num):
		self.bus.write_byte(self.MUX_ADDR, (1<<bus_num))
		self.currentBus = bus_num
		
	def getBatteryTemp(self):
		if(self.currentBus != 3):
			try:
				self.openBus(3)
			except IOError:
				return 0
		try:
			data = self.Therm.get_obj_temp_C()
		except IOError:
			data = 0
		return data
		
	def getOrientation(self, imu_num):
		if(self.currentBus != 1):
			try:
				self.openBus(1)
			except IOError:
				return [0, 0, 0]
		if(imu_num == 1):
			try:
				data = self.IMU1.read_euler()
			except IOError:
				data = [0, 0, 0]
			return data
		if(imu_num == 2):
			try:
				data = self.IMU2.read_euler()
			except IOError:
				data = [0, 0, 0]
			return data
		else:
			print("Illegal Selection")
			return 0
		
	def getAcceleration(self, imu_num):
		if(self.currentBus != 1):
			try:
				self.openBus(1)
			except IOError:
				return 0
		if(imu_num == 1):
			try:
				data = self.IMU1.read_linear_acceleration()
			except IOError:
				return 0
			return data[0] * self.METER2G
		if(imu_num == 2):
			try:
				data = self.IMU2.read_linear_acceleration()
			except IOError:
				return 0
			return data[0] * self.METER2G
		else:
			print("Illegal Selection")
			return 0		
		
	def getLidarDistance(self):
		if(self.currentBus != 0):
			try:
				self.openBus(0)
			except IOError:
				return 0
		try:
			data = self.Lidar.getDistance()
		except IOError:
			data = 0
		return data
		
	def getTubePressure(self):
		if(self.currentBus != 1):
			try:
				self.openBus(1)
			except IOError:
				return 0
		try:
			data = self.BMP.read_pressure()
		except IOError:
			data = 0
		return data
		
	def getTubeTemp(self):
		if(self.currentBus != 1):
			try:
				self.openBus(1)
			except IOError:
				return 0
		try:
			data = self.BMP.read_temperature()
		except IOError:
			data = 0
		return data
	
	def getBMEpressure(self, vessel):
		if(vessel == 1):
			if(self.currentBus != 3):
				try:
					self.openBus(3)
				except IOError:
					return 0
			try:
				t1 = self.BME1.read_raw_temp()
				data = self.BME1.read_pressure()
			except IOError:
				data = 0
			return data
		if(vessel == 2):
			if(self.currentBus != 2):
				try:
					self.openBus(2)
				except IOError:
					return 0
			try:
				t1 = self.BME2.read_raw_temp()
				data = self.BME2.read_pressure()
			except IOError:
				data = 0
			return data
		else:
			print("Illegal vessel")
			data = 0
			return data
			
	def getBMEtemperature(self, vessel):
		if(vessel == 1):
			if(self.currentBus != 3):
				try:
					self.openBus(3)
				except IOError:
					return 0
			try:
				data = self.BME1.read_temperature()
			except IOError:
				return 0
			return data
		if(vessel == 2):
			if(self.currentBus != 2):
				try:
					self.openBus(2)
				except IOError:
					return 0
			try:
				data = self.BME2.read_temperature()
			except IOError:
				return 0
			return data
		else:
			print("Illegal Selection")
			return 0
			
	def getVoltageLevel(self):
		if(self.currentBus != 3):
			try:
				self.openBus(3)
			except IOError:
				return 0
		try:
			data = self.ADC.read_adc(self.VOLT, self.ADC_GAIN)
		except IOError:
			data = 0
		return data * self.ADC_CONVERT
			
	def getCurrentLevel(self):
		if(self.currentBus != 3):
			try:
				self.openBus(3)
			except IOError:
				return 0
		try:
			data = self.ADC.read_adc(self.AMP, self.ADC_GAIN)
		except IOError:
			data = 0
		return data * self.ADC_CONVERT
		
		
	
		
if __name__ == '__main__':
	system = HyperlynxECS()
	system.initializeSensors()
	
	while True:
		distance = system.getLidarDistance()
		battTemp = system.getBatteryTemp()
		accel1 = system.getAcceleration(1)
		accel2 = system.getAcceleration(2)
		orient1 = system.getOrientation(1)
		orient2 = system.getOrientation(2)
		temp1 = system.getBMEtemperature(1)
		temp2 = system.getBMEtemperature(2)
		press1 = system.getBMEpressure(1)
		press2 = system.getBMEpressure(2)
		tubepress = system.getTubePressure()
		tubetemp = system.getTubeTemp()
		voltage = system.getVoltageLevel()
		current = system.getCurrentLevel()
		print("%.2f cm\t"%distance, "%.2f C\t"%battTemp, "%.2f G\t"%accel1, "%.2f G"%accel2)
		print("X: %.2f\t"%orient1[0], "Y: %.2f\t"%orient1[1], "Z: %.2f"%orient1[2])
		print("X: %.2f\t"%orient2[0], "Y: %.2f\t"%orient2[1], "Z: %.2f"%orient2[2])
		print("PV1: %.2f psi\t"%press1, "%.2f C\t"%temp1, "PV2: %.2f psi\t"%press2, "%.2f"%temp2)
		print("Tube: %.2f psi\t"%tubepress, "%.2f C"%tubetemp)
		print("%.2f V\t"%voltage, "%.2f A"%current)
		print("\n")
		sleep(0.1)

