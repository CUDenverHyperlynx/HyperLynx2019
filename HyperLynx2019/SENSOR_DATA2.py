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
from time import sleep, clock
from mlx90614 import MLX90614
from Adafruit_BNO055 import BNO055
from Adafruit_BME280 import BME280

bus = smbus.SMBus(1)

#I2C ADDRESSES FOR SENSORS

IR_ADDRESS = 0x5B
BNO_ADDRESS_A = 0x28
BNO_ADDRESS_B = 0x29
BME280_ADDRESS_A = 0x77
BME280_ADDRESS_B = 0x76

PASC2PSI = 6894.757  #TO CONVERT PASCALS TO PSI
METER2FEET = 3.281  #TO CONVERT METERS TO FEET
readings = 9 #Sensor values read each loop

DATA_READ = 0.004 #time between readings, in seconds

#INITIALIZE SENSORS

IR_Therm = MLX90614(IR_ADDRESS)
IMU_Nose = BNO055.BNO055(None, BNO_ADDRESS_A)
IMU_Tail = BNO055.BNO055(None, BNO_ADDRESS_B)
PV1 = BME280(address=BME280_ADDRESS_A)
PV2 = BME280(address=BME280_ADDRESS_B)
"""
#For some brilliant reason the BME280 python library requires that
#temp be read before pressure can be read as the read_raw_temp function
#initializes an attribute required for all other readings
t1 = PV1.read_temperature()
t2 = PV2.read_temperature()
#BNO055 LIBRARY REQUIRES self.begin
IMU_Nose.begin()
IMU_Tail.begin()
"""
"""
sensorSetup:
    Checks all sensors for connection to I2C bus
    Displays error if no connection
"""
def sensorSetup():
    if(not IMU_Nose.begin()):
        print("ERROR CONNECTING TO BNO055 AT 0x28")
        return False
    else:
        print("BNO055 Nose Ready")
        sleep(0.05)
    if(not IMU_Tail.begin()):
        print("ERROR CONNECTING TO BNO055 AT 0X29")
        return False
    else:
        print("BNO055 Tail Ready")
        sleep(0.05)
    try:
        bus.write_quick(IR_ADDRESS)
        print("MLX90614 Ready")
        sleep(0.05)
    except IOError:
        print("ERROR CONNECTING TO MLX90614 AT 0X5B")
        return False
    try:
        bus.write_quick(BME280_ADDRESS_A)
        print("BME280 PV1 Ready")
        sleep(0.05)
    except IOError:
        print("ERROR CONNECTING TO BME280 AT 0x77")
        return False
    try:
        bus.write_quick(BME280_ADDRESS_B)
        print("BME280 PV2 Ready")
        sleep(0.05)
    except IOError:
        print("ERROR CONNECTING TO BME280 AT 0x76")
        return False
    PV1.read_temperature()
    PV2.read_temperature()
    return True

"""
medainFilter:
    Filters outliers from data readings to avoid false flags
    from signal noise. The sensors will populate into a 5 item
    list, then be sorted numerically and only the median value
    will be returned by the function.
"""
def medianFilter(data):
    data.sort()
    return data[2]
"""
sensorData:
    Returns a tuple of filtered sensor readings at a rate of 30Hz
    [0]           Pressure from BME820 @ ADDRESS_A
    [1]           Pressure from BME280 @ ADDRESS_B
    [2]           Battery temperature in degrees Celsius from MLX90614 @ IR_ADDRESS
    [3]           Linear acceleration X direction from BNO055 @ ADDRESS_A
    [4]           Linear acceleration X direction from BNO055 @ ADDRESS_B
    [5][0]        X oreintation from BNO055 @ ADDRESS_A
    [5][1]        Y orientation from BNO055 @ ADDRESS_A
    [5][2]        Z orientation from BNO055 @ ADDRESS_A
    [6][0]        X orientation from BNO055 @ ADDRESS_B
    [6][1]        Y orientation from BNO055 @ ADDRESS_B
    [6][2]        Z orientation from BNO055 @ ADDRESS_B
    [7]           Pressure vessel 1 temperature in degrees Celsius from BME280 @ ADDRESS_A
    [8]           Pressure vessel 2 temperature in degrees Celsius from BME280 @ ADDRESS_B
    [9]           Time to poll sensors in seconds
    [10]          IOErrors in current sensor poll
"""
def sensorData():
    #Initialize fo poll time
    start = clock()
    #Initialize to count errors
    fault = 0
    #Initialize empty lists to populate raw sensor data
    pressure_1 = []
    pressure_2 = []
    PV1_temp = []
    PV2_temp = []
    batt_temp = []
    accel_1 = []
    accel_2 = []
    noseX = []
    noseY = []
    noseZ = []
    tailX = []
    tailY = []
    tailZ = []
              
    for x in range(0, 5):
        #Poll each sensor and append reading to designated list
        #In case of IOError, append 0 and increment fault
        try:
            orientationDataNose = IMU_Nose.read_euler()
        except IOError:
            orientationDataNose = [0, 0, 0]
            fault+=1
        noseX.append(orientationDataNose[0])
        noseY.append(orientationDataNose[1])
        noseZ.append(orientationDataNose[2])
        try:            
            orientationDataTail = IMU_Tail.read_euler()
        except IOError:
            orientationDataTail = [0, 0, 0]
            fault+=1
        tailX.append(orientationDataTail[0])
        tailY.append(orientationDataTail[1])
        tailZ.append(orientationDataTail[2])
        try:
            accelerationDataNose = IMU_Nose.read_linear_acceleration()
        except IOError:
            accelerationDataNose = [0, 0, 0]
            fault+=1
        accel_1.append(accelerationDataNose[0])
        try:
            accelerationDataTail = IMU_Tail.read_linear_acceleration()
        except IOError:
            accelerationDataTail = [0, 0, 0]
            fault+=1
        accel_2.append(accelerationDataTail[0])
        try:
            batt_temp.append(IR_Therm.get_obj_temp_C())
        except IOError:
            batt_temp.append(0)
            fault+=1
        try:
            pressure_1.append(PV1.read_pressure())
        except IOError:
            pressure_1.append(0)
            fault+=1
        try:
            pressure_2.append(PV2.read_pressure())
        except IOError:
            pressure_2.append(0)
            fault+=1
        try:
            PV1_temp.append(PV1.read_temperature())
        except IOError:
            PV1_temp.append(0)
            fault+=1
        try:
            PV2_temp.append(PV2.read_temperature())
        except IOError:
            PV2_temp.append(0)
            fault+=1
        sleep(DATA_READ)
    #Filter each list and store median value to be displayed
    pressureVessel_1 = medianFilter(pressure_1) / PASC2PSI
    pressureVessel_2 = medianFilter(pressure_2) / PASC2PSI
    batteryTemp = medianFilter(batt_temp)
    noseAcceleration = medianFilter(accel_1) * METER2FEET
    tailAcceleration = medianFilter(accel_2) * METER2FEET
    noseOrientation = (medianFilter(noseX), medianFilter(noseY), medianFilter(noseZ))
    tailOrientation = (medianFilter(tailX), medianFilter(tailY), medianFilter(tailZ))
    PV1TEMP = medianFilter(PV1_temp)
    PV2TEMP = medianFilter(PV2_temp)

    end = clock()
    runTime = start - end
    return(pressureVessel_1, pressureVessel_2, batteryTemp, noseAcceleration, tailAcceleration, noseOrientation, tailOrientation, PV1TEMP, PV2TEMP, runTime, fault)

if __name__ == '__main__':
    if(sensorSetup()):
        sleep(0.5)
        error = 0
        attempts = 0
        while True:
            data = sensorData()
            error += data[10]
            attempts += readings
            print("Battery Temp:\t%.2f C" % data[2])
            print("PV1:\tPressure:\t%.2f psi\t" % data[0], "Temp:\t%.2f C" % data[7])
            print("PV2:\tPressure:\t%.2f psi\t" % data[1], "Temp:\t%.2f C" % data[8])
            print("NOSE:\tAcceleration:\t%.2f ft/s^2" %data[3])
            print("X:\t%.2f\t" % data[5][0], "Y:\t%.2f\t" % data[5][1], "Z:\t%.2f" %data[5][2])
            print("Tail:\tAcceleration:\t%.2f ft/s^2" % data[4])
            print("X:\t%.2f\t" % data[6][0], "Y:\t%.2f\t" % data[6][1], "Z:\t%.2f" % data[6][2])
            print("Run Time:\t%.2f" % data[9])
            print("Current Poll Success:\t%.2f" % (((readings - data[10])/readings)*100))
            print("Overall Success Rate:\t%.2f" % (((attempts -error)/attempts)*100))
            print("\n")
