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
from time import sleep
from time import time
import sys
import math
import cmath
import serial
import socket
import pickle
import ctypes
import struct
import numpy


class Status():
    Fault = 0
    SafeToApproach = 1
    FlightControlToLaunch = 2
    Launching = 3
    Coasting = 4  # Unused
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 5
    table_of_ranges = numpy.genfromtxt(r'abortranges.dat', skip_header=1, delimiter=',', usecols=numpy.arange(1,10))


check_aborts = Status.table_of_ranges
print(check_aborts)


def poll_sensors():
    return


if __name__ == "__main__":
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
