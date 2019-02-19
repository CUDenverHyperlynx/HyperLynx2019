"""
Author:       Patrick Tafoya
Purpose:      Test sensors for pod control systems
              The loop reads and prints all pertinent
              data at a rate of 30Hz
              
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
METER2FEET = 3.281
DATA_READ = 0.005 #time between readings, in seconds
DATA_DISP = 0.01/3 #0.00333333; sleep time between displays(5 sensors * 0.005s + 0.0053333s = 30Hz display)

#INITIALIZE SENSORS
IR_Therm = MLX90614(IR_ADDRESS)
IMU_Nose = BNO055.BNO055(None, BNO_ADDRESS_A)
IMU_Tail = BNO055.BNO055(None, BNO_ADDRESS_B)

PV1 = BME280(address=BME280_ADDRESS_A)
PV2 = BME280(address=BME280_ADDRESS_B)
#For some brilliant reason the BME280 python library requires that
#temp be read before pressure can be read as the read_raw_temp function
#initializes an attribute required for all other readings
t1 = PV1.read_temperature()
t2 = PV2.read_temperature()
#BNO055 LIBRARY REQUIRES self.begin
IMU_Nose.begin()
IMU_Tail.begin()       

"""
Medain Filter:
       Filters outliers from data readings to avoid false flags
       from signal noise. The sensors will populate into a 5 item
       list, then be sorted numerically and only the median value
       will be returned by the function.
"""
def medianFilter(data):
       data.sort()
       return data[2]

"""
Sensor Data:
       Returns a tuple of filtered sensor readings at a rate of 30Hz
       [0]           Pressure from BME820 @ ADDRESS_A
       [1]           Pressure from BME280 @ ADDRESS_B
       [2]           Temperature in degrees Celsius from MLX90614 @ IR_ADDRESS
       [3]           Linear acceleration X direction from BNO055 @ ADDRESS_A
       [4]           Linear acceleration X direction from BNO055 @ ADDRESS_B
       [5][0]        X oreintation from BNO055 @ ADDRESS_A
       [5][1]        Y orientation from BNO055 @ ADDRESS_A
       [5][2]        Z orientation from BNO055 @ ADDRESS_A
       [6][0]        X orientation from BNO055 @ ADDRESS_B
       [6][1]        Y orientation from BNO055 @ ADDRESS_B
       [6][2]        Z orientation from BNO055 @ ADDRESS_B
"""

def sensorData():
       #Initialize empty lists to populate raw sensor data
              pressure_1 = []
              pressure_2 = []
              temp = []
              accel_1 = []
              accel_2 = []
              noseX = []
              noseY = []
              noseZ = []
              tailX = []
              tailY = []
              tailZ = []
       
              for x in range(0, 5):
                     #Variables to store on reading of each sensor to store into list
                     orientationDataNose = IMU_Nose.read_euler()
                     orientationDataTail = IMU_Tail.read_euler()
                     accelerationDataNose = IMU_Nose.read_linear_acceleration()
                     accelerationDataTail = IMU_Tail.read_linear_acceleration()
                     tempData = IR_Therm.get_obj_temp_C()
                     PV_Data_1 = PV1.read_pressure()
                     PV_Data_2 = PV2.read_pressure()
                     #Store each reading into a list for filtering
                     pressure_1.append(PV_Data_1)
                     pressure_2.append(PV_Data_2)
                     temp.append(tempData)
                     #BNO055 returns data in tuples (x, y, z) so
                     #we need to store each element in separate lists
                     accel_1.append(accelerationDataNose[0])
                     accel_2.append(accelerationDataTail[0])
                     noseX.append(orientationDataNose[0])
                     noseY.append(orientationDataNose[1])
                     noseZ.append(orientationDataNose[2])
                     tailX.append(orientationDataTail[0])
                     tailY.append(orientationDataTail[1])
                     tailZ.append(orientationDataTail[2])
                     #pause to allow required time in between readings
                     time.sleep(DATA_READ)
                     
              #Filter each list and store median value to be displayed
              pressureVessel_1 = medianFilter(pressure_1) / PASC2PSI
              pressureVessel_2 = medianFilter(pressure_2) / PASC2PSI
              batteryTemp = medianFilter(temp)
              noseAcceleration = medianFilter(accel_1) * METER2FEET
              tailAcceleration = medianFilter(accel_2) * METER2FEET
              noseOrientation = (medianFilter(noseX), medianFilter(noseY), medianFilter(noseZ))
              tailOrientation = (medianFilter(tailX), medianFilter(tailY), medianFilter(tailZ))
              
              return(pressureVessel_1, pressureVessel_2, batteryTemp, noseAcceleration, tailAcceleration, noseOrientation, tailOrientation)

if __name__ == '__main__':
       while True:
          
              data = sensorData()
              print("Temp:\t%.2f C" % data[2])
              print("PV1:\t%.2f" % data[0])
              print("PV2:\t%.2f" % data[1])
              print("NOSE:\tAcceleration:\t%.2f ft/s^2" %data[3])
              print("X:\t%.2f\t" % data[5][0], "Y:\t%.2f\t" % data[5][1], "Z:\t%.2f" %data[5][2])
              print("Tail:\tAcceleration:\t%.2f ft/s^2" % data[4])
              print("X:\t%.2f\t" % data[6][0], "Y:\t%.2f\t" % data[6][1], "Z:\t%.2f" % data[6][2])
              print("\n")
              time.sleep(DATA_DISP)   
                     
                     
