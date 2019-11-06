"""
			Author:		Patrick Tafoya
			Purpose:	Monitor pod health through I2C sensor
						network and control all IO triggers on Rpi
						for CMOS switches.

			Hardware List:
				TCA9548 I2C Multiplexer (1)
					-Create multiple I2C channels
					-Allow duplicate addresses
					-Manage parallel pull-up resistance

				PCA90614 I2C Differential (4)
					-Negate parasitic capacitance and noise
						on I2C wires across long distance

				LIDAR Lite V3 I2C Lidar Distance Sensor (1)
					-End of tube detection during final crawl

				BMP280 I2C Pressure/Temperature/Humidity Sensor (1)
					-Ambient pressure and temperature of tube

				BME280 I2C Pressure/Temperature/Humidity Sensor (2)
					-Pressure and temperature in each pressure vessel

				MLX90614 IR Temperature Sensor (1)
					-Temperature of LV battery
				BNO055 9DOF Sensor (2)

					-Linear Acceleration
					-X, Y, Z Orientation

				ADS1115 16-bit Analog to Digital Converter
					-Convert analog signals to digital I2C signals
					-Brake pressure transducer
					-Current sensor for LV battery
					-voltage sensor for LV battery

				Arduino Micro as I2C Slave
					-Stripe count from SICK laser sensors

			Multiplexer Bus Delegation
			bus 0:
				Location:	NOSE
				Voltage:	5V
				PCA Diff:	Yes
					LIDAR Lite V3	addr 0x62
					BMP280 			addr 0x77
					Arduino Micro	addr 0x5F
			bus 1:
				Location: 	Left PV
				Voltage:	3.3V
				PCA Diff:	Yes
					BME280			addr 0x76
			bus 2:
				Location:	Right PV1
				Voltage:	3.3V
				PCA Diff:	No
					MLX90614		addr 0x5B
					ADS1115			addr 0x48
			bus 3:
				Location:	RPV2
				Voltage:	3.3V
				PCA Diff:	No
					BME280			addr 0x77
					BNO055			addr 0x29
					BNO055			addr 0x28
		
		POD ORIENTATION AXIS
					
					|Y   / Z
					|   /
					|  /
					| /
					|/
		X-----------
"""
import smbus
from time import sleep, clock
from mlx90614 import MLX90614
from Adafruit_BNO055 import BNO055
import Adafruit_ADS1x15
from HyperlynxBMP280 import BMP280
from Adafruit_BME280 import BME280
import Lidar
import RPi.GPIO
import os
import math	#cos()

class HyperlynxECS():
	def __init__(self, bus_num=1):
		#Default I2C Addresses for Each Sensor
		self.MUX_ADDR = 0x70
		self.IR_ADDR = 0x5B
		self.IMU_ADDR1 = 0x28
		self.IMU_ADDR2 = 0x29
		self.BMP_ADDR = 0x77
		self.BME_ADDR1 = 0x77
		self.BME_ADDR2 = 0x76
		self.LID_ADDR = 0x62
		self.ADC_ADDR = 0x48
		self.MICRO_ADDR = 0x5F
		#Open I2C Bus
		self.bus = smbus.SMBus(bus_num)
		#Assign Sensor Locations to Multiplexer Channels
		self.tcaNOSE = 0
		self.tcaPVL = 1
		self.tcaPVR = 2
		self.tcaPVR2 = 3
		#Monitor Current Open I2C Bus (10 is ALL CLOSED)
		self.currentBus = 10
		#Maximum Attempts to Connect
		self.connectAttempt = 5
		#Initialize Stripe Count to Zero
		self.STRIPE_COUNT = 0
		#Conversion Factors for Acceleration, Analog Sensors and ADC
		self.METER2G = (3.28084/32.2)
		self.PASC2PSI = (1 / 6894.757)
		self.VOLT2PSI = 94.697
		self.ADC_CONVERT = (4.096 / 32767.0)
		self.vRatio = 4.13
		self.iRatio = 1
		#Assign ADC Pins for Analog Sensors
		self.PRESSURE = 2
		self.VOLT = 0
		self.AMP = 1
		#Set ADC Gain for 4.096V Max
		self.ADC_GAIN = 1
		#Initialize GPIO Control-Use BCM Pin Numbering-Disable Warnings for Pin Mode Overide
		self.IO = RPi.GPIO
		self.IO.setmode(self.IO.BCM)
		self.IO.setwarnings(False)
		#Assign DROK CMOS Signal Switches to Rpi IO Pin Numbers
		#Solenoids for Braking System
		self.NOsolPIN = 17
		self.NCsol1PIN = 27
		self.NCsol2PIN = 22
		#Motor Controller Coolant Pump
		self.CoolPumpPIN = 5
		#HV COntactors and Warning LEDs (Red LED Tied to Contactors)
		self.greenPIN = 6
		self.contactorPIN1 = 13
		self.contactorPIN2 = 18
		#Active Low Reset for IR Thermometer
		self.MLXrstPIN = 19
		#Active Low Reset Count (For Testing and Debugging)
		self.MLXRST = 0
		#Initialize Orientation Offset Angles for IMUs to Zero
		self.X1OFFSET = 0
		self.X2OFFSET = 0
		self.Z1OFFSET = 0
		self.Z2OFFSET = 0
		#Sensor Status Monitoring for NACK Recieved to Filter True Data
		self.MLX_status = False
		self.BNO1_status = False
		self.BNO2_status = False
		self.BME1_status = False
		self.BME2_status = False
		self.BMP_status = False
		self.LID_status = False
		self.ADC_status = False
		self.TCA_status = False
		self.MICRO_status = False
		#Establish Connection to I2C Multiplexer
		try:
			#Attempt to Connect to Multiplexer
			self.bus.write_byte(self.MUX_ADDR, 0)
			print("TCA I2C Multiplexer Ready")
			self.TCA_status = True
		except IOError:
			#If Connection Fails Stop Module
			print("Connection Error with TCA I2C Multiplexer")
			while True:
				pass
		#Close All Multiplexer Channels
		self.closeAllBus()
		
	"""initializeSensors()
			-Establish Connection to All Sensors
			-No Parameters
	"""
	def initializeSensors(self):
		try:
			#Attempt to Open Pod Nose Multiplexer Channel
			#	-Successful Conection => Continue
			#	-Unsuccessful Connection => Wait and Retry
			for x in range(0, self.connectAttempt):
				self.openBus(self.tcaNOSE)
				if(self.currentBus == self.tcaNOSE):
					break
				sleep(0.5)
		#Unsuccessful After Max Attempts
		except IOError:
			print("Connection error with TCA Multiplexer")
			self.TCA_status = False
			#return 0
		#Attempt Connection and Initialization of Lidar
		#	-Create Object
		#	-Test Write to Verify Connection
		try:
			self.Lidar = Lidar.Lidar_Lite()
			self.bus.write_quick(self.LID_ADDR)
			#print("Lidar Ready")
			self.LID_status = True
		except IOError:
			print("Connection error with Lidar")
			#return 0
		# Attempt Connection to BMP280
		#	-Create Object
		#	-Attempt Reading to Verify Connection
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.BMP = BMP280(address=self.BMP_ADDR)
				if(self.BMP.read_raw_temp()):
					#print("BMP Ready")
					self.BMP_status = True
					break
				sleep(0.5)
		#Unsuccessful After Max Attempts
		except IOError:
			print("Connection Error with BMP")
			#return 0
		# Attempt Connection to Arduino Micro
		#	-Test Read to Verify Connection
		try:
			for x in range(0, self.connectAttempt):
				self.bus.read_byte(self.MICRO_ADDR)
				print("Arduino Micro Ready")
				self.MICRO_status = True
				break
		except IOError:
			print("Connection Error with Arduino Micro")
			#return 0
		# Attempt to Open Left PV Multiplexer Channel
		#	-Successful Conection => Continue
		#	-Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.openBus(self.tcaPVL)
				if(self.currentBus == self.tcaPVL):
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection error wit TCA Multiplexer")
			self.TCA_status = 0
			#return 0
		# Attempt Connection to BME280
		#	-Create Object
		#	-Attempt Reading to Verify Connection
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.BMEL = BME280(address=self.BME_ADDR2)
				if(self.BMEL.read_raw_temp()):
					#print("BME LPV Ready")
					self.BME2_status = True
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection Error with BME2 LPV")
			#return 0
		# Attempt to Open Right PV Multiplexer Channel One
		#	-Successful Conection => Continue
		#	-Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.openBus(self.tcaPVR)
				if(self.currentBus == self.tcaPVR):
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection error with TCA Multiplexer")
			self.TCA_status = False
			#return 0
		# Attempt Connection to MLX90614
		#	-Create Object
		#	-Attempt Reading to Verify Connection
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.Therm = MLX90614(address=self.IR_ADDR)
				if(self.Therm.check_connect):
					print("MLX90614 Ready")
					self.MLX_status = True
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection Error with Therm")
			#return 0
		# Attempt Connection to ADS1115
		#	-Create Object
		try:
			self.ADC = Adafruit_ADS1x15.ADS1115(address=self.ADC_ADDR, busnum=1)
			print("ADC Ready")	
			self.ADC_status = True
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection error with ADS ADC")
			#return 0
		# Attempt to Open Right PV Multiplexer Channel Two
		#	-Successful Conection => Continue
		#	-Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.openBus(self.tcaPVR2)
				if(self.currentBus == self.tcaPVR2):
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection error with TCA Multiplexer")
			self.TCA_status = False
			#return 0
		# Attempt Connection to BME280
		#	-Create Object
		#	-Attempt Reading to Verify Connection
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.BMER = BME280(address=self.BME_ADDR1)
				if(self.BMER.read_raw_temp()):
					print("BME RPV Ready")
					self.BME1_status = True
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection Error with BME RPV")
			#return 0
		# Attempt Connection to BNO055 One
		#	-Create Object
		#	-Set Mode (Magnometer Disabled)
		#	-Calibrate Orientation
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.IMU1 = BNO055(address=self.IMU_ADDR1)
				if(self.IMU1.begin(mode=0x08)):
					while(self.IMU1.get_calibration_status()[1] != 3):
						pass
					print("IMU1 Ready")
					self.BNO1_status = True
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection error with IMU1")
			#return 0
		#Set Offset Angles for Accurate Calculations
		try:
			self.X1OFFSET = self.getOrientation(1)[1]
			print("IMU1 Y offset angle set")
		except IOError:
			print("IMU1 Y offset angle not set")
		try:
			self.Z1OFFSET = self.getOrientation(1)[2]
			print("IMU1 Z offset angle set")
		except IOError:
			print("IMU1 Z offset angle not set")
		# Attempt Connection to BNO055 One
		#	-Create Object
		#	-Set Mode (Magnometer Disabled)
		#	-Calibrate Orientation
		#	-If Unsuccessful Connection => Wait and Retry
		try:
			for x in range(0, self.connectAttempt):
				self.IMU2 = BNO055(address=self.IMU_ADDR2)
				if(self.IMU2.begin(mode=0x08)):
					while(self.IMU2.get_calibration_status()[1] != 3):
						pass
					print("IMU2 Ready")
					self.BNO2_status = True
					break
				sleep(0.5)
		# Unsuccessful After Max Attempts
		except IOError:
			print("Connection Error with IMU2")
			#return 0
		# Set Offset Angles for Accurate Calculations
		try:
			self.X2OFFSET = self.getOrientation(2)[1]
			print("IMU2 Y offset angle set")
		except IOError:
			print("IMU2 Y offset angle not set")
		try:
			self.Z2OFFSET = self.getOrientation(2)[2]
			print("IMU2 Z offset angle set")
		except IOError:
			print("IMU2 Z offset angle not set")
		return 1

	"""closeAllBus()
		-No parameters
		-Closes all channels on Multiplexer
		-Updates current open bus
	"""
	def closeAllBus(self):
		self.bus.write_byte(self.MUX_ADDR, 0)
		self.currentBus = 10

	"""openBus()
		-Parameters:	bus_num - desired channel (0-7)
		-Open desired channel on multiplexer
		-Updates current open bus
	"""
	def openBus(self, bus_num):
		self.bus.write_byte(self.MUX_ADDR, (1<<bus_num))
		self.currentBus = bus_num\

	"""getBatteryTemp()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates IR Therm and Multiplexer Status
		-Returns LV battery temperature in degrees Celsius
	"""
	def getBatteryTemp(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.Therm.get_obj_temp_C()
			self.MLX_status = True
		except IOError:
			data = 0
			self.MLX_status = False
		return data
	"""getOrientation()
		-Parameters:	imu_num - desired IMU (1 or 2)
		-Checks open bus, opens if necessary
		-Updates BNO055 and Multiplexer status
		-Adjusts orientation reading according to offset angles
		-Returns Tuple of x, y, and z orientation in degrees
	"""
	def getOrientation(self, imu_num):
		if(self.currentBus != self.tcaPVR2):
			try:
				self.openBus(self.tcaPVR2)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return [0, 0, 0]
		if(imu_num == 1):
			try:
				data = self.IMU1.read_euler()
				self.BNO1_status = True
			except IOError:
				data = [0, 0, 0]
				self.BNO1_status = False
			y = data[0]
			x = data[1] - self.X1OFFSET
			z = data[2] - self.Z1OFFSET
			return [x, y, z]
		elif(imu_num == 2):
			try:
				data = self.IMU2.read_euler()
				self.BNO2_status = True
			except IOError:
				data = [0, 0, 0]
				self.BNO2_status = False
			y = data[0]
			x = data[1] - self.X2OFFSET
			z = data[2] - self.Z2OFFSET
			return [x, y, z]
		else:
			#print("Illegal Selection")
			return 0

	"""getAcceleration()
		-Parameters:	imu_num - desired IMU (1 or 2)
		-Checks open bus, opens if necessary
		-Updates BNO055 and Multiplexer status
		-Converts from ms^-2 to G's
		-Returns Tuple of x, y, and z linear acceleration
	"""
	def getAcceleration(self, imu_num):
		if(self.currentBus != self.tcaPVR2):
			try:
				self.openBus(self.tcaPVR2)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return (0, 0, 0)
		if(imu_num == 1):
			try:
				data = self.IMU1.read_linear_acceleration()
				z = data[0] * self.METER2G
				x = data[1] * self.METER2G
				y = data[2] * self.METER2G
				self.BNO1_status = True
			except IOError:
				self.BNO1_status = False
				return (0, 0, 0)
			return (z, x, y)
		elif(imu_num == 2):
			try:
				data = self.IMU2.read_linear_acceleration()
				z = data[0] * self.METER2G
				x = data[1] * self.METER2G
				y = data[2] * self.METER2G
				self.BNO2_status = True
			except IOError:
				self.BNO2_status = False
				return (0, 0, 0)
			return (z, x, y)
		else:
			#print("Illegal Selection")
			return 0

	"""getLidarDistance()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates Lidar and Multiplexer status
		-Returns distance in feet
	"""
	def getLidarDistance(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.Lidar.getDistance()
			self.LID_status = True
		except IOError:
			data = 0
			self.LID_status = False
		return data * 0.0328084

	"""getTubePressure()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates BMP280 and Multiplexer status
		-Returns pressure in PSI
	"""
	def getTubePressure(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.BMP.read_pressure()
			self.BMP_status = True
		except IOError:
			data = 0
			self.BMP_status = False
		return data
	"""getTubeTemp()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates BMP280 and Multiplexer status
		-Returns temperature in degrees Celsius
	"""
	def getTubeTemp(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.BMP.read_temperature()
			self.BMP_status = True
		except IOError:
			data = 0
			self.BMP_status = False
		return data

	"""getBMEpressure()
		-Parameters	vessel - desired PV (1 = Right, 2 = Left)
		-Checks open bus, opens if necessary
		-Updates BME280 and Multiplexer status
		-Returns pressure in PSI
	"""
	def getBMEpressure(self, vessel):
		if(vessel == 1):
			if(self.currentBus != self.tcaPVR2):
				try:
					self.openBus(self.tcaPVR2)
					self.TCA_status = True
				except IOError:
					self.TCA_status = 0
					return 0
			try:
				data = self.BMER.read_pressure()
				self.BME1_status = True
			except IOError:
				data = 0
				self.BME1_status = False
			return data	* self.PASC2PSI
		elif(vessel == 2):
			if(self.currentBus != self.tcaPVL):
				try:
					self.openBus(self.tcaPVL)
					self.TCA_status = True
				except IOError:
					self.TCA_status = False
					return 0
			try:
				data = self.BMEL.read_pressure()
				self.BME2_status = True
			except IOError:
				data = 0
				self.BME2_status = False
			return data * self.PASC2PSI
		else:
			print("Illegal vessel")

	"""getBMEtemperature
		-Parameters	vessel - desired PV (1 = Right, 2 = Left)
		-Checks open bus, opens if necessary
		-Updates BME280 and Multiplexer status
		-Returns temperature in degrees Celsius
	"""
	def getBMEtemperature(self, vessel):
		if(vessel == 1):
			if(self.currentBus != self.tcaPVR2):
				try:
					self.openBus(self.tcaPVR2)
					self.TCA_status = True
				except IOError:
					self.TCA_status = False
					return 0
			try:
				data = self.BMER.read_temperature()
				self.BME1_status = True
			except IOError:
				data = 0
				self.BME1_status = False
			return data
		elif(vessel == 2):
			if(self.currentBus != self.tcaPVL):
				try:
					self.openBus(self.tcaPVL)
					self.TCA_status = True
				except IOError:
					self.TCA_status = False
					return 0
			try:
				data = self.BMEL.read_temperature()
				self.BME2_status = True
			except IOError:
				self.BME2_status = False
				return 0
			return data
		else:
			#print("Illegal Selection")
			return 0
	"""getVoltageLevel()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates ADC and Multiplexer status
		-Returns LV battery voltage in Volts
	"""
	def getVoltageLevel(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.ADC.read_adc(self.VOLT, self.ADC_GAIN)
			self.ADC_status = True
		except IOError:
			data = 0
			self.ADC_status = False
		return (data-4700) * self.ADC_CONVERT * self.vRatio

	"""getCurrentLevel()#DOES NOT WORK YET, SOURCING NEW SENSOR	
		-No parameters
		-Checks open bus, opens if necessary
		-Updates ADC and Multiplexer status
		-Returns current in Amps
	"""
	def getCurrentLevel(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.ADC.read_adc(self.AMP, self.ADC_GAIN)
			self.ADC_status = True
		except IOError:
			data = 0
			self.ADC_status = False
		return (data-4700) * self.ADC_CONVERT * self.iRatio

	"""getBrakePressure()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates ADC and Multiplexer status
		-Returns brake pressure in PSI
	"""
	def getBrakePressure(self):
		if(self.currentBus != self.tcaPVR):
			try:
				self.openBus(self.tcaPVR)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
		try:
			data = self.ADC.read_adc(self.PRESSURE, self.ADC_GAIN)
			self.ADC_status = True
		except IOError:
			data = 0
			self.ADC_status = False
		return (data-4700) * self.ADC_CONVERT * self.VOLT2PSI

	"""getStripeCount()
		-No parameters
		-Checks open bus, opens if necessary
		-Updates Arduino Micro and Multiplexer status
		-Returns stripe count tally from SICK lasers
	"""
	def getStripeCount(self):
		if(self.currentBus != self.tcaNOSE):
			try:
				self.openBus(self.tcaNOSE)
				self.TCA_status = True
			except IOError:
				self.TCA_status = False
				return 0
			try:
				self.STRIPE_COUNT = self.bus.read_byte(self.MICRO_ADDR)
				self.MICRO_status = True
			except IOError:
				self.MICRO_status = False
			return self.STRIPE_COUNT

	"""initializeIO()
		-No parameters
		-Sets all DROCK triggers to proper Rpi IO Pins
	"""
	def initializeIO(self):
		self.IO.setup(self.contactorPIN1, self.IO.OUT, initial=self.IO.LOW)#SET CONTACTOR1 PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.greenPIN, self.IO.OUT, initial=self.IO.LOW)	#SET GREEN LED PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.contactorPIN2, self.IO.OUT, initial=self.IO.LOW)	#SET CONTACTOR 2 PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NOsolPIN, self.IO.OUT, initial=self.IO.LOW)	#SET NO SOLENOID PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NCsol1PIN, self.IO.OUT, initial=self.IO.LOW)	#SET NC SOLENOID RES 1 PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.NCsol2PIN, self.IO.OUT, initial=self.IO.LOW)	#SET NC SOLENOID RES 2 PIN AS OUTPUT, INITIALIZE LOW
		self.IO.setup(self.CoolPumpPIN, self.IO.OUT, initial=self.IO.LOW)#SET COOLANT PUMP PIN TO OUTPUT, INITIALIZE LOW
		self.IO.setup(self.MLXrstPIN, self.IO.OUT, initial=self.IO.HIGH)

	"""switchGreenLED()
		-Parameters:	status - desired level (0 = LOW, 1 = HIGH)
		-Sets Green LED IO pin to desired level
	"""
	def switchGreenLED(self, status):
		if(status == 0):
			self.IO.output(self.greenPIN, self.IO.LOW)
		else:
			self.IO.output(self.greenPIN, self.IO.HIGH)
			
	"""switchSolenoid()
		-Parameters:	solenoid(1 = NC res 1, 2 = NC res 2, 3 = NO), status(0 = LOW, 1 = HIGH)
		-Sets selected solenoid to desired IO level
	"""
	def switchSolenoid(self, solenoid, status):
		if(solenoid == 1):
			if(status == 0):
				self.IO.output(self.NCsol1PIN, self.IO.LOW)
			elif(status == 1):
				self.IO.output(self.NCsol1PIN, self.IO.HIGH)
		elif(solenoid == 2):
			if(status == 0):
				self.IO.output(self.NCsol2PIN, self.IO.LOW)
			elif(status == 1):
				self.IO.output(self.NCsol2PIN, self.IO.HIGH)
		elif(solenoid == 3):
			if(status == 0):
				self.IO.output(self.NOsolPIN, self.IO.LOW)
			elif(status == 1):
				self.IO.output(self.NOsolPIN, self.IO.HIGH)
	"""switchCoolantPump()
		-Parameters:	status - desired IO level(0 = LOW, 1 = HIGH)
		-Switches coolant pump on and off
	"""
	def switchCoolantPump(self, status):
		if(status == 0):
			self.IO.output(self.CoolPumpPIN, self.IO.LOW)
		else:
			self.IO.output(self.CoolPumpPIN, self.IO.HIGH)

	"""switchContactor()
		-Parameters:	contactor(1 = Contactor 1; 2 = Contactor 2), status - desired level(0 = LOW; 1 = HIGH)
		-Sets selected HV contactor to desired level
		-Red LED will light when both contactors set high
	"""
	def switchContactor(self, contactor, status):
		if(contactor == 1):
			if(status == 0):
				self.IO.output(self.contactorPIN1, self.IO.LOW)
			elif(status == 1):
				self.IO.output(self.contactorPIN1, self.IO.HIGH)
		elif(contactor == 2):
			if(status == 0):
				self.IO.output(self.contactorPIN2, self.IO.HIGH)
			elif(status == 1):
				self.IO.output(self.contactorPIN2, self.IO.HIGH)
	"""MLX_RESET()
		-No parameters
		-Resets IR Thermometer with active low reset
		-Pulls low for 10us then sets high again
	"""
	def MLX_RESET(self):
		self.IO.output(self.MLXrstPIN, self.IO.LOW)
		sleep(0.00001)
		self.IO.output(self.MLXrstPIN, self.IO.HIGH)

	"""statusCheck()
		-No parameters
		-Checks Status of all sensors
		-Resets IR thermometer if necessary
			*IR therm crashed I2C bus during testing, this fixes all resulting issues
	"""
	def statusCheck(self):
		data = self.MLX_status + self.BNO1_status + self.BNO2_status + self.BME1_status + self.BME2_status + self.BMP_status + self.LID_status + self.ADC_status + self.TCA_status + self.MICRO_status
		if(data == 0):
			self.MLX_RESET()
			self.MLXRST = self.MLXRST + 1
			self.TCA_status = True
		return (data / 10) * 100
				
if __name__ == '__main__':
	system = HyperlynxECS()
	system.initializeIO()
	if(system.initializeSensors()):
		while True:
			startTime = clock()
			distance = system.getLidarDistance()
			tubepress = system.getTubePressure()
			tubetemp = system.getTubeTemp()
			battTemp = system.getBatteryTemp()
			temp2 = system.getBMEtemperature(2)
			press2 = system.getBMEpressure(2)
			voltage = system.getVoltageLevel()
			current = system.getCurrentLevel()
			brakePressure = system.getBrakePressure()
			accel1 = system.getAcceleration(1)
			accel2 = system.getAcceleration(2)
			orient1 = system.getOrientation(1)
			orient2 = system.getOrientation(2)
			temp1 = system.getBMEtemperature(1)
			press1 = system.getBMEpressure(1)
			stripes = system.getStripeCount()
			stat = system.statusCheck()
			endTime = clock() - startTime
			print(stat)
			print(system.MLXRST)
			print(endTime)
			print("%.2f\t"%distance)
			print("%.2f C\t"%battTemp, "%.2f G\t"%accel1[0], "%.2f G"%accel2[0])
			print("X: %.2f\t"%orient1[0], "Y: %.2f\t"%orient1[1], "Z: %.2f"%orient1[2])
			print("X: %.2f\t"%orient2[0], "Y: %.2f\t"%orient2[1], "Z: %.2f"%orient2[2])
			print("PVL: %.2f psi\t"%press1, "%.2f C\t"%temp1, "PVR: %.2f psi\t"%press2, "%.2f C"%temp2)
			print("Tube: %.2f psi\t"%tubepress, "%.2f C"%tubetemp)
			print("%.2f V\t"%voltage, "%.2f A"%current)
			print("Brakes: %.2f psi" % brakePressure)
			temp = os.popen("vcgencmd measure_temp").readline()
			print(temp)
			print("\n")
			#sleep(0.05)
