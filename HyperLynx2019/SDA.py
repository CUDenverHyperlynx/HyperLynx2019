'''
   HyperLynx: SDA.py

   Purpose:
   Provide motor controller proper state determined by this algorithm and sensor information
   Python version of the SDA

   State Determination Algorithm

   State       Description
   0000-0      Fault
   0001-1      Safe to Approach(Standby)
   0010-2      Flight Control to Launch
   0011-3      Launching(Accel 1 - Top Speed)
   0100-4      Coasting(NOT USED)
   0101-5      Braking(High Speed)
   0110-6      Crawling(Accel 2 - Low Speed)
   0111-7      Braking(Low Speed)

   I/O Parameters:

   INPUTS
   v(t)-Pod Velocity(0-400 ft/s)
   a(t)-Pod Acceleration(0-10 G)
   B(t)-Pneumatic Brake System State(1-CLOSED/ON/VENT 0-OPEN/OFF/CHARGED)
   d(t)-Pod Position in Tube(distance traveled)(0-4150 ft)
   E(t)-Individual Sensor Error Flags(0-NO ERROR 1-ERROR FLAG)

      - E(t) = (LV Current, LV Voltage, LV Temp., BME1, BME2, BMP, IMU's, BMS Current,
                BMS Voltage, BMS Temp., Pneumatics, MC Current, MC Voltage, MC Temp., Comms Loss, Spare)

   OUTPUTS
   HVC(t)-High Voltage Contactors(0-CLOSED/SHORT/ON 1-OPEN/OFF)
   TR(t) -Throttle Control(MAX/CRAWL/OFF)
   LG(t) -Data Logging
   B(t)  -Pneumatic Brake System Control(1-CLOSED/ON/VENT 0-OPEN/OFF/CHARGED)

   SENSORS:

   Purpose:
   Pull in sensor readings

   Details:
   BBP = Preflight Variable
   2 IMUs (9DoF)
   2 PV (BME280)
   1 IR Temp
   1 Current and Voltage
   2 Laser Range Sensors (1 Left, 1 Right)
   1 PV (Pneumatics)
   3 MC Reads
   3 BMS Reads
   1 Uno

   WhoToBlame:
   John Brenner
'''

#Trying to get changes correct

from argparse import ArgumentParser
from time import sleep, time
# import sys
# import math
# import cmath
# import serial
import socket
# import pickle
# import ctypes
# import struct
import numpy
from subprocess import call

class Status():
    # ACTUAL POD STATES, NOT SPACEX STATES
    Fault = False
    Trigger = False
    SafeToApproach = 1
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7
    Quit = False

    def __init__(self):
        self.state = self.SafeToApproach
        self.abort_labels = numpy.genfromtxt('abortranges.dat', dtype=str, skip_header=1, usecols=0, delimiter='\t')
        self.abort_ranges = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 13))
        #print(self.abort_ranges)
        print("Pod init complete, State: " + str(self.state))

def poll_sensors():
    PodStatus.sensor_data = numpy.genfromtxt('fake_sensor_data.txt', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 3))
    #print(PodStatus.sensor_data)
    return

def eval_abort():
    # temp_range column values
        #   0 - Sensor ID
        #   1 - Low Value
        #   2 - High Value
        #   3 - Safe To Approach bool
        #   4 - FC2L bool
        #   5 - Launch bool
        #   6 - Brake1 bool
        #   7 - Postflight bool
        #   8 - Brake2 bool
        #   9 - Trigger bool
        #   10 - Fault bool

    if PodStatus.Fault == True:
        abort()

    a = numpy.shape(PodStatus.abort_ranges) # Load abort_ranges size into tuple, so we can iterate any array size

    if PodStatus.state == PodStatus.SafeToApproach:   # Evaluates abort criteria for Safe To Approach state
        for i in range(a[0]):
            temp_range = PodStatus.abort_ranges[i]    # Loads current abort range
            temp_sensor = PodStatus.sensor_data[i]    # Loads current sensor data
            if temp_range[3] == 1:                    # Evaluates the S2A column value
                if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
                    PodStatus.abort_ranges[i,11] = 1
                    # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
                    print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
                          "\tValue: " + str(temp_sensor[1]) +
                          "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
                elif temp_sensor[1] > temp_range[2]:    # Evaluates "HIGH" values
                    PodStatus.abort_ranges[i,11] = 1
                    # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
                    print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
                          "\tValue: " + str(temp_sensor[1]) +
                          "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
                else:
                    PodStatus.abort_ranges[i,11] = 0

    if PodStatus.state == PodStatus.Launching:        # Evaluates abort criteria for FC2L state
        for i in range(a[0]):
            temp_range = PodStatus.abort_ranges[i]    # Loads current sensor range to temp
            temp_sensor = PodStatus.sensor_data[i]
            if temp_range[5] == 1:                    # Evaluates the FC2L column value
                if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
                    PodStatus.abort_ranges[i,11] = 1
                    # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
                    print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
                          "\tValue: " + str(temp_sensor[1]) +
                          "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
                elif (temp_sensor[1] > temp_range[2]):    # Evaluates "HIGH" values
                    PodStatus.abort_ranges[i,11] = 1
                    # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
                    print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
                          "\tValue: " + str(temp_sensor[1]) +
                          "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
                else:
                    PodStatus.abort_ranges[i,11] = 0

    # EVALUATE IF ANY FAULT CONDITIONS EXIST
    temp_fault_count = 0
    for i in range(a[0]):
        temp_fault_count += PodStatus.abort_ranges[i,11]
    if temp_fault_count > 0:
        PodStatus.Fault = True
        print("Current faults: " + str(temp_fault_count))
    else: PodStatus.Fault = False

    temp_trigger_count = 0
    for i in range(a[0]):
        temp_trigger_count += PodStatus.abort_ranges[i, 10]
    if temp_trigger_count > 0:
        print("Current triggers: " + str(temp_trigger_count))
        PodStatus.Trigger = True
    else:
        PodStatus.Trigger = False
    return

def send_data():
    parser = ArgumentParser(description="Hyperlynx POD Run")
    parser.add_argument("--team_id", type=int, default=0, help="HyperLynx id to send")
    parser.add_argument("--frequency", type=int, default=30, help="The frequency for sending packets")
    parser.add_argument("--server_ip", default="192.168.0.1", help="The ip to send packets to")
    parser.add_argument("--server_port", type=int, default=3000, help="The UDP port to send packets to")
    parser.add_argument("--tube_length", type=int, default=4150, help="Total length of the tube(ft)")
    parser.add_argument("--bbp", type=int, default=3228, help="Begin Braking Point(ft)")
    parser.add_argument("--cbp", type=int, default=4060, help="Crawling Brake Point(ft)")
    parser.add_argument("--topspeed", type=int, default=396, help="Top Speed in ft/s")
    parser.add_argument("--crawlspeed", type=int, default=30, help="Crawl Speed in ft/s")
    parser.add_argument("--time_run_highspeed", type=int, default=15, help="Run Time in seconds")

    args = parser.parse_args()

    if args.frequency < 10:
        print("Send frequency should be higher than 10Hz")
    if args.frequency > 50:
        print("Send frequency should be lower than 50Hz")

    team_id = args.team_id
    wait_time = (1 / args.frequency)
    server = (args.server_ip, args.server_port)

    tube_length = args.tube_length
    time_run_highspeed = args.time_run_highspeed

    top_speed = args.topspeed
    BBP = args.bbp

    crawl_speed = args.crawlspeed
    CBP = args.cbp

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    status = Status.SafeToApproach

    seconds = 0

def run_state():

    if PodStatus.state == 1:          # S2A STATE FUNCTIONS
        a = input("type 1 to go")
        if a == '1':
            transition()
            return
        return
    elif PodStatus.state == 3:
        PodStatus.Quit = True                # LAUNCH STATE FUNCTIONS
    elif PodStatus.state == 4:
        pass
        # DO STUFF
    elif PodStatus.state == 5:
        pass
        # DO STUFF
    elif PodStatus.state == 6:
        pass
        # DO STUFF
    elif PodStatus.state == 7:
        pass
        # DO STUFF
    else:
        PodStatus.Fault = True


def transition():
    if PodStatus.state == 1:          # S2A trans
        PodStatus.state = 3
        print("TRANS: S2A(1) to LAUNCH(3)")
    elif PodStatus.state == 3:          # LAUNCH trans
        PodStatus.state = 5
        print("TRANS: LAUNCH(3) TO BRAKE(5)")
    elif PodStatus.state == 5:
        PodStatus.state = 7
        print("TRANS: BRAKE(5) to S2A(1)")
    elif PodStatus.state == 6:
        PodStatus.state = 7
        print("TRANS: CRAWLING(6) TO BRAKE(7)")
    elif PodStatus.state == 7:
        PodStatus.state = 1
        print("TRANS: BRAKE(7) TO S2A(1)")
    else:
        print("POD IN INVALID STATE: " + str(PodStatus.state))
        PodStatus.state = 1
        PodStatus.Fault = True
    return

def abort():
    if PodStatus.state == 1:          # S2A STATE FUNCTIONS
        pass
    elif PodStatus.state == 3:          # LAUNCH STATE FUNCTIONS
        PodStatus.state = 7
    elif PodStatus.state == 5:
        PodStatus.state = 7
    elif PodStatus.state == 6:
        PodStatus.state = 7
    elif PodStatus.state == 7:
        pass
    else:
        PodStatus.state = 1

if __name__ == "__main__":

    PodStatus = Status()

    while PodStatus.Quit == False:
        print("State: " + str(PodStatus.state))
        print("Fault: " + str(PodStatus.Fault))
        poll_sensors()
        run_state()
        eval_abort()
        send_data()
    print("Quitting")
    #call("sudo shutdown -h now", shell=True)
