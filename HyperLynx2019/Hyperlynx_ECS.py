"""
Author:		Patrick Tafoya
Purpose:	Test full ECS
			For this test i have the sensors hooked up to the following tca channels
			
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
		self.MUX_ADDR = 0x70											#I2C ADDRESS FOR TCA9548A MULTIPLEXER
		self.IR_ADDR = 0x5B												#I2C ADDRESS FOR MLX90614 IR THERMOMETER
		self.IMU_ADDR1 = 0x29											#I2C ADDRESS 1 FOR BNO055 ABSOLUTE ORIENTATION
		self.IMU_ADDR2 = 0x28											#I2C ADDRESS 2 FOR BNO055 ABSOLUTE ORIENTATION
		self.BMP_ADDR = 0x77											#I2C ADDRESS FOR BMP280 PRESSURE SENSOR
		self.BME_ADDR1 = 0x77											#I2C ADDRESS 1 FOR BME280 PRESSURE SENSOR
		self.BME_ADDR2 = 0x76											#I2C ADDRESS 2 FOR BME280 PRESSURE SENSOR
		self.LID_ADDR = 0x62											#I2C ADDRESS FOR LIDAR LITE V3
		self.ADC_ADDR = 0x48											#I2C ADDRESS FOR ADS1115 ADC
		self.METER2G = (3.28084/32.2)									#CONVERSION: METER TO FEET => FEET/G FOR ACCELERATION
		self.VOLT2PSI = 59.761											#CONVERSION: VOLTS READ BY ADC TO PSI FOR HONEYWELL PRESSURE SENSOR
		self.PRESSURE = 2												#ADC PIN FOR HONEYWELL PRESSURE SENSOR
		self.VOLT = 0													#ADC PIN FOR ATTOPILOT VOLTAGE READ
		self.AMP = 1													#ADC PIN FOR ATTOPILOT CURRENT READ
		self.ADC_GAIN = 1												#GAIN SETTING FOR ADC: READS UP TO 4.096 V
		self.ADC_CONVERT = (4.096/32767.0)								#CONVERT ADC 16BIT SIGNED RESOLUTION TO VOLTAGE LEVEL FOR GAIN OF 1
		self.vRatio = 4.13												#ATTOPILOT RATIO: VOLTAGE TRANSMITTED TO ACTUAL VOLTAGE READING
		self.iRatio = 1													#ATTOPILOT RATIO: VOLTAGE TRANSMITTED TO ACTUAL CURRENT READING
		self.IO = RPi.GPIO												#INITIALIZE RPi.GPIO LIBRARY TO CONTROL DROK SWITCHES
		self.NOsolPIN = 17												#DROK SIGNAL PIN FOR NORMALLY OPEN SOLENOID
		self.NCsol1PIN = 27												#DROK SIGNAL PIN FOR NORMALLY CLOSED SOLENOID RESERVIOR 1
		self.NCsol2PIN = 22												#DROK SIGNAL PIN FOR NORMALLY CLOSED SOLENOID RESERVOIR 2
		self.CoolPumpPIN = 5											#DROK SIGNAL PIN FOR COOLANT PUMP
		self.greenPIN = 6												#DROK SIGNAL PIN FOR GREEN LED
		self.redPIN = 13												#DROK SIGNAL PIN FOR RED LED
		self.bus = smbus.SMBus(bus_num)									#OPEN I2C BUS
		self.closeAllBus()												#RESET TCA9548A MULTIPLEXER TO CLOSE ALL CHANNELS AT STARTUP
		self.IO.setmode(self.IO.BCM)									#BCM MODE USES BROADCOM SOC CHANNEL NUMBER FOR EACH PIN
		self.IO.setwarnings(False)										#TURN OFF WARNINGS TO ALLOW OVERIDE OF CURRENT GPIO CONFIGURATION
		self.IO.setup(self.greenPIN, self.IO.OUT, initial=self.IO.LOW)	#SET GREEN LED PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.redPIN, self.IO.OUT, initial=self.IO.LOW)	#SET RED LED PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NOsolPIN, self.IO.OUT, initial=self.IO.HIGH)
		self.IO.setup(self.NCsol1PIN, self.IO.OUT, initial=self.IO.LOW)
		self.IO.setup(self.NCsol2PIN, self.IO.OUT, initial=self.IO.LOW)
		self.currentBus = 10											#VARIABLE TO KEEP TRACK OF WHICH TCA CHANNEL IS OPEN, 10 WILL BE NO CHANNEL AS 0 IS A SPECIFIC CHANNEL
		self.tcaLIDAR = 0												#TCA CHANNEL FOR LIDAR
		self.tcaNOSE = 1												#TCA CHANNEL FOR NOSE AVIONICS
		self.tcaPVL = 2													#TCA CHANNEL FOR LEFT PV AVIONICS
		self.tcaPVR = 3													#TCA CHANNEL FOR RIGHT PV AVIONICS
		
	"""SETUP AL AVIONICS SENSORS ON THE I2C BUS"""
	def initializeSensors(self):
		self.openBus(self.tcaLIDAR)										#OPEN CHANNEL ON TCA
		self.Lidar = Lidar.Lidar_Lite()									#CREATE OBJECT FOR LIDAR LITE V3
		try:
			self.bus.write_quick(self.LID_ADDR)							#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("Lidar Ready")
		except IOError:
			print("Connection error with Lidar")						#PRINT ERROR IF UNABLE TO CONNECT
		self.openBus(self.tcaNOSE)										#OPEN CHANNEL ON TCA
		self.IMU1 = BNO055(address=self.IMU_ADDR1)						#CREATE OBJECT FOR BNO055 AT ADDRESS 1
		if(self.IMU1.begin()):											#CHECK FOR CONNECTION TO I2C BUS AND SET OPERATION MODE
			print("IMU1 Ready")
		else:
			print("Connection Error with IMU1")
		self.IMU2 = BNO055(address=self.IMU_ADDR2)						#CREATE OBJECT FOR BNO055 AT ADDRESS 2
		if(self.IMU2.begin()):											#CHECK FOR CONNECTION TO I2C BUS AND SET OPERATION MODE
			print("IMU2 Ready")
		else:
			print("Connection Error with IMU2")
		self.BMP = BMP280(address=self.BMP_ADDR)						#CREATE OBJECT FOR BMP280
		try:
			self.bus.write_quick(self.BMP_ADDR)							#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("BMP Ready")
		except IOError:
			print("Connection Error with BMP")
		self.openBus(self.tcaPVL)										#OPEN CHANNEL ON TCA
		self.BMEL = BME280(address=self.BME_ADDR2)						#CREATE OBJECT FOR BME280 AT ADDRESS 2
		try:
			self.bus.write_quick(self.BME_ADDR2)						#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("BME2 Ready")
		except IOError:
			print("Connection Error with BME2")
		self.openBus(self.tcaPVR)										#OPEN TCA CHANNEL
		self.Therm = MLX90614(address=self.IR_ADDR)						#CREATE OBJECT FOR MLX90614 IR THERMOMETER
		try:
			self.bus.write_quick(self.IR_ADDR)							#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("Therm Ready")
		except IOError:
			print("Connection Error with Therm")
		self.ADC = Adafruit_ADS1x15.ADS1115(address=self.ADC_ADDR, busnum=1)	#CREATE OBJECT FOR ADS1115 ADC 
		try:
			self.bus.write_quick(self.ADC_ADDR)							#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("ADC Ready")
		except IOError:
			print("Connection Error with ADC")
		self.BMER = BME280(address=self.BME_ADDR1)						#CREATE  OBJECT FOR BME280 AT ADDRESS 2
		try:
			self.bus.write_quick(self.BME_ADDR1)						#QUICK WRITE TO TEST CONNECTION TO I2C BUS
			print("BME1 Ready")
		except IOError:
			print("Connection Error with BME1")
			
	"""CLOSE ALL CHANNELS ON THE TCA"""
	def closeAllBus(self):
		self.bus.write_byte(self.MUX_ADDR, 0)							#WRITE ALL BITS ON TCA CHANNEL SELECT REGISTER LOW
		self.currentBus = 10											#UPDATE CURRENT OPEN CHANNEL
	"""OPEN SPECIFIC CHANNEL ON THE TCA"""
	def openBus(self, bus_num):											#PARAMETER IS DESIRED TCA CHANNEL TO OPEN
		self.bus.write_byte(self.MUX_ADDR, (1<<bus_num))				#WRITE SPECIFIED CHANNEL BIT HIGH ON TCA SELECT REGISTER
		self.currentBus = bus_num										#UPDATE CURRENT OPEN CHANNEL
	"""FETCH LV BATTERY TEMPERATURE"""	
	def getBatteryTemp(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)								#CHECKS FOR CURRENT OPEN CHANNEL TO SAVE TIME SELECTING IF IT IS ALREADY OPEN, IF ALREADY NOT OPENED, OPEN IT
			except IOError:
				return 0												#RETURNS A ZERO IF CANNOT CONNECT TO THE TCA
		try:
			data = self.Therm.get_obj_temp_C()							#FETCH OBJECT TEMP
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO MLX90614
		return data														#RETURNS TEMPERATURE READING IN DEGREES CELSIUS
	"""FETCH ORIENTATION"""												#PARAMETER IS DESIRED IMU (1 OR 2)
	def getOrientation(self, imu_num):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)								#CHECK IF BUS IS OPENED, IF IT IS NOT, OPEN IT
			except IOError:
				return [0, 0, 0]										#RETURNS TUPLE OF ZEROS IF CANNOT CONNECT TO TCA
		if(imu_num == 1):
			try:
				data = self.IMU1.read_euler()							#READS X, Y AND Z ORIENTATION IN DEGREES AND RETURNS AS TUPLE
			except IOError:
				data = [0, 0, 0]										#SETS AS ZEROS IF CANNOT CONNECT TO BNO055
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
	"""FETCH LINEAR ACCELERATION"""										#PARAMETER IS DESIRED IMU (1 OR 2)										
	def getAcceleration(self, imu_num):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
			except IOError:
				return 0
		if(imu_num == 1):
			try:
				data = self.IMU1.read_linear_acceleration()
			except IOError:
				return 0
			return data[0] * self.METER2G								#RETURNS ACCELERATION IN G'S
		if(imu_num == 2):
			try:
				data = self.IMU2.read_linear_acceleration()
			except IOError:
				return 0
			return data[0] * self.METER2G								#RETURNS ACCELERATION IN G'S
		else:
			print("Illegal Selection")
			return 0		
	"""FETCH LIDAR DISTANCE READING"""										
	def getLidarDistance(self):
		if(self.currentBus != self.tcaLIDAR):
			try:
				self.openBus(self.tcaLIDAR)
			except IOError:
				return 0												#RETURNS ZERO IF CANNOT CONNECT TO TCA
		try:
			data = self.Lidar.getDistance()								#FETCH DISTANCE
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO LIDAR
		return data / 100												#RETURNS DISTANCE IN METERS
	"""FETCH TUBE PRESSURE"""	
	def getTubePressure(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
			except IOError:
				return 0												#RETURNS ZERO IF CANNOT CONNECT TO TCA
		try:
			data = self.BMP.read_pressure()								#FETCH PRESSURE
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO BMP
		return data														#RETURNS PRESSURE IN PSI
	"""FETCH TUBE TEMPERATURE"""	
	def getTubeTemp(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
			except IOError:
				return 0												#RETURNS ZERO IF CANNOT CONNECT TO TCA
		try:
			data = self.BMP.read_temperature()							#FETCH TEMPERATURE
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO BMP
		return data														#RETURNS TEMPERATURE IN DEGREES CELSIUS
	"""FETCH BME PRESSURE"""											#PARAMETER IS DESIRED BME (1 = RIGHT PV, 2 = LEFT PV)
	def getBMEpressure(self, vessel):
		if(vessel == 1):
			if(self.currentBus != self.tcaPVR):
				try:
					self.openBus(self.tcaPVR)
				except IOError:
					return 0											#RETURNS ZERO IF CANNOT CONNECT TO TCA
			try:
				t1 = self.BMER.read_raw_temp()							#BME REQUIRES A TEMP READ TO INITIALIZE VARIABLES FOR ALL OTHER READINGS. WILL CHANGE THIS TO initializeSensors()
				data = self.BMER.read_pressure()						#FETCH PRESSURE
			except IOError:
				data = 0												#SETS AS ZERO IF CANNOT CONNECT TO BME
			return data													#RETURNS PRESSURE IN PSI
		if(vessel == 2):
			if(self.currentBus != self.tcaPVL):
				try:
					self.openBus(self.tcaPVL)
				except IOError:
					return 0
			try:
				t1 = self.BMEL.read_raw_temp()
				data = self.BMEL.read_pressure()
			except IOError:
				data = 0
			return data
		else:
			print("Illegal vessel")
			data = 0
			return data
	"""FETCH BME TEMPERATURE"""											#PARAMETER IS DESIRED BME (1 = RIGHT PV, 2 = LEFT PV) 		
	def getBMEtemperature(self, vessel):
		if(vessel == 1):
			if(self.currentBus != self.tcaPVR):
				try:
					self.openBus(self.tcaPVR)
				except IOError:
					return 0											#RETURNS ZERO IF CANNOT CONNECT TO TCA
			try:
				data = self.BMER.read_temperature()						#FETCH TEMPERATURE
			except IOError:
				data = 0												#SETS AS ZERO IF CANNOT CONNECT TO BME
			return data													#RETURNS TEMPERATURE IN DEGREES CELSIUS
		if(vessel == 2):
			if(self.currentBus != self.tcaPVL):
				try:
					self.openBus(self.tcaPVL)
				except IOError:
					return 0
			try:
				data = self.BMEL.read_temperature()
			except IOError:
				return 0
			return data
		else:
			print("Illegal Selection")
			return 0
	"""FETCH LV BATTERY VOLTAGE LEVEL"""		
	def getVoltageLevel(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
			except IOError:
				return 0												#RETURNS ZERO IF CANNOT CONNECT TO TCA
		try:
			data = self.ADC.read_adc(self.VOLT, self.ADC_GAIN)			#READ VOLTAGE PIN SET ON ADC
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO ADC
		return data * self.ADC_CONVERT * self.vRatio					#CONVERTS ADC BITS TO ACTUAL VOLTAGE SENT BY ATTOPILOT AND SCALES TO VOLTAGE READ BY ATTOPILOT, RETURNS VOLTAGE
	"""FETCH LV BATTERY CURRENT DRAW"""#DOES NOT WORK YET		
	def getCurrentLevel(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
			except IOError:
				return 0
		try:
			data = self.ADC.read_adc(self.AMP, self.ADC_GAIN)
		except IOError:
			data = 0
		return data * self.ADC_CONVERT * self.iRATIO
	"""FETCH BRAKE LINE PRESSURE"""	
	def getBrakePressure(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
			except IOError:
				return 0												#RETURNS ZERO IF CANNOT CONNECT TO TCA
		try:
			data = self.ADC.read_adc(self.PRESSURE, self.ADC_GAIN)		#READ PRESSURE PIN SET ON ADC
		except IOError:
			data = 0													#SETS AS ZERO IF CANNOT CONNECT TO ADC
		return data * self.ADC_CONVERT * self.VOLT2PSI					#CONVERTS ADC BITS TO ACTUAL VOLTAGE SENT BY HONEYWELL AND CONVERTS VOLTAGE TO PSI, RETURNS PRESSURE IN PSI
	
	def initializeDROK():
		self.IO.setup(self.greenPIN, self.IO.OUT, initial=self.IO.LOW)	#SET GREEN LED PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.redPIN, self.IO.OUT, initial=self.IO.LOW)	#SET RED LED PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NOsolPIN, self.IO.OUT, initial=self.IO.HIGH)	#SET NO SOLENOID PIN AS OUTPUT, INITIALIZE HIGH
		self.IO.setup(self.NCsol1PIN, self.IO.OUT, initial=self.IO.LOW)	#SET NC SOLENOID RES 1 PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NCsol2PIN, self.IO.OUT, initial=self.IO.LOW)	#SET NC SOLENOID RES 2 PIN AS OUTPUT, INITIALIZE LOW
			
	def switchGreenLED(self, status):
		if(status == 1):
			self.IO.output(self.greenPIN, self.IO.HIGH)
		else:
			self.IO.output(self.greenPIN, self.IO.LOW)
			
	def switchRedLED(self, status):
		if(status == 1):
			self.IO.output(self.redLED, self.IO.HIGH)
		else:
			self.IO.output(self.redLED, self.IO.LOW)
				
	
		
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
		brakePressure = system.getBrakePressure()
		print("%.2f m\t"%distance, "%.2f C\t"%battTemp, "%.2f G\t"%accel1, "%.2f G"%accel2)
		print("X: %.2f\t"%orient1[0], "Y: %.2f\t"%orient1[1], "Z: %.2f"%orient1[2])
		print("X: %.2f\t"%orient2[0], "Y: %.2f\t"%orient2[1], "Z: %.2f"%orient2[2])
		print("PV1: %.2f psi\t"%press1, "%.2f C\t"%temp1, "PV2: %.2f psi\t"%press2, "%.2f"%temp2)
		print("Tube: %.2f psi\t"%tubepress, "%.2f C"%tubetemp)
		print("%.2f V\t"%voltage, "%.2f A"%current)
		print("Brakes: %.2f psi" % brakePressure)
		print("\n")
		sleep(0.1)

