"""
Author:       Patrick Tafoya
Purpose:      Test sensors for pod control systems
              The loop reads and prints all pertinent
              data every half-second
              
              Sensors connected via RPi I2C Bus
              RPi pin 1     3.3V
              RPi pin 3     SDA
              RPi pin 5     SCL
              RPi pin 9     GND
              
              IR THERM
              A5 is SCL
              A4 is SDA
              
              BNO055_ADDRESS_2 requires additional 3.3V to ADR
"""
import smbus
import time
from mlx90614 import MLX90614
from Adafruit_BNO055 import BNO055
from Adafruit_BME280 import BME280

#I2C ADDRESSES FOR SENSORS
IR_ADDRESS = 0x5B
BNO_ADDRESS_A = 0x28
BNO_ADDRESS_B = 0x29
BME280_ADDRESS_A = 0x77
BME280_ADDRESS_B = 0x76
PASC2PSI = 6894.757  #TO CONVERT PASCALS TO PSI
DATA_READ = 0.005 #time between readings, in seconds
DATA_DISP = 0.005333333 #sleep time between displays(5 sensors * 0.005s + 0.0053333 = 30Hz display)

#INITIALIZE SENSORS
IR_Therm = MLX90614(IR_ADDRESS)
IMU_Nose = BNO055.BNO055(None, BNO_ADDRESS_A)
IMU_Tail = BNO055.BNO055(None, BNO_ADDRESS_B)

PV1 = BME280(address=BME280_ADDRESS_A)
PV2 = BME280(address=BME280_ADDRESS_B)
#For some brilliant reason the BME280 python library requires that
#temp be read before pressure can be read as the read_raw_temp function
#initializes an attribute required for all other readings"""
t1 = PV1.read_temperature()
t2 = PV2.read_temperature()
#BNO055 LIBRARY REQUIRES self.begin
IMU_Nose.begin()
IMU_Tail.begin()

"""
       Filter Functions to filter noise from data readings.
       Each function will create a list for the sensor.
       It will then add 5 readings into the list, spaced
       DATA_READ seconds apart. (Exact time TBD)
       Next it runs the list through a median filter:
              data = (15, 15, 66, 1, 99) becomes data = (1, 15, 15, 16, 99)
              returns median value of sorted list, effectively negating any noise
"""
def tempFilter(sensor):
       tempMatrix = []
       for x in range(0, 5):
              tempMatrix.append(sensor.get_obj_temp_C())
              time.sleep(DATA_READ)
       return medianFilter(tempMatrix)

def pressureFilter(sensor):
       pressureMatrix = []
       for x in range(0, 5):
              pressureMatrix.append(sensor.read_pressure())
              time.sleep(DATA_READ)
       return (medianFilter(pressureMatrix) / PASC2PSI)#convert from Pascal to PSI
       
def orientFilter(sensor):
       #BNO055 returns orientation as a tuple(x, y, z) so three matrices are required
       xMatrix = []
       yMatrix = []
       zMatrix = []
       for x in range(0, 5):
              #store tuple
              orientData = sensor.read_euler()
              #add appropriate vectors to the right matrix
              xMatrix.append(orientData[0])
              yMatrix.append(orientData[1])
              zMatrix.append(orientData[2])
              time.sleep(DATA_READ)
       x = medianFilter(xMatrix)
       y = medianFilter(yMatrix)
       z = medianFilter(zMatrix)
       #returns as a tuple
       return(x, y, z)
       
def accelerationFilter(sensor):
       #BNO055 returns acceleration as a tuple as well, but we only want x direction
       accelMatrix = []
       for x in range(0, 5):
              #Store tuple
              acceleration = sensor.read_linear_acceleration()
              #Store x reading into matrix
              accelMatrix.append(acceleration[0])
              time.sleep(DATA_READ)
       return (medianFilter(accelMatrix) * 3.281)#convert from meters to feet
       

def medianFilter(data):
       data.sort()
       return data[2]

if __name__ == '__main__':

    while True:
       pressure_1 = pressureFilter(PV1)
       pressure_2 = pressureFilter(PV2)
       Temp = tempFilter(IR_Therm)
       Nose = orientFilter(IMU_Nose)
       Tail = orientFilter(IMU_Tail)
       Accel_1 = accelerationFilter(IMU_Nose)
       Accel_2 = accelerationFilter(IMU_Tail)
       print("TEMP:\t%.2f C" % Temp)
       print("PV1:\t%.2f psi" % pressure_1)
       print("PV2:\t%.2f psi" % pressure_2)
       print("NOSE:\tACCELERATION:\t%.2f f/s^2" % Accel_1)
       print("X: %.2f\t" % Nose[0], "Y: %.2f\t" % Nose[1], "Z: %.2f" % Nose[2])
       print("TAIL\tACCELERATION:\t%.2f f/s^2" % Accel_2)
       print("X: %.2f\t" % Tail[0], "Y: %.2f\t" % Tail[1], "Z: %.2f\n\n" % Tail[2])
       time.sleep(DATA_DISP)
