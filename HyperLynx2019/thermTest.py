import smbus
import time
from mlx90614 import MLX90614
from Adafruit_BNO055 import BNO055

BNO_ADDR_A = 0x28
IR_ADDR = 0x5B
SLEEPTIME = 0.01
therm = MLX90614(IR_ADDR)
IMU_Nose = BNO055.BNO055(None, BNO_ADDR_A)
bus = smbus.SMBus(1)

def medianFilter(data):
    data.sort
    return data[2]

def filteredData():
    rawTemp = []
    noseX = []
    noseY = []
    noseZ = []
    accel = []
    for x in range(0, 5):
        try:
            rawTemp.append(therm.get_obj_temp_C())
        except IOError:
            rawTemp.append(0)
        try:
            noseOrientation = IMU_Nose.read_euler()
        except IOError:
            noseOrientation = [0, 0, 0]
        noseX.append(noseOrientation[0])
        noseY.append(noseOrientation[1])
        noseZ.append(noseOrientation[2])
        try:
            acceleration = IMU_Nose.read_linear_acceleration()
            accel.append(acceleration[0])
        except IOError:
            accel.append(0)
            
        time.sleep(SLEEPTIME)
    temp = medianFilter(rawTemp)
    x = medianFilter(noseX)
    y = medianFilter(noseY)
    z = medianFilter(noseZ)
    acc = medianFilter(accel) * 3.281
    return (temp, x, y, z, acc)

if __name__ == '__main__':
    IMU_Nose.begin()
    while True:
        data = filteredData()
        print("%.2f C" % data[0])
        print("%.2f\t" % data[1], "%.2f\t" % data[2], "%.2f\t" % data[3])
        print("%.2f m/s^2" % data[4])
        time.sleep(0.05)
     
