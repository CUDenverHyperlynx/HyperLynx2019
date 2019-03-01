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
   John Brenner & Jeff Stanek
'''

from argparse import ArgumentParser
from time import sleep, clock
import socket, struct
import numpy
import datetime
import os

class Status():
    # States
    SafeToApproach = 1
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7

    abort_ranges = {}
    abort_ranges[SafeToApproach] = {}
    abort_ranges[Launching] = {}
    abort_ranges[BrakingHigh] = {}
    abort_ranges[Crawling] = {}
    abort_ranges[BrakingLow] = {}
    commands = {}
    sensor_data = {}

    def __init__(self):
        # BOOT FUNCTIONS
        self.StartTime = clock()
        self.HV = 0
        self.Brakes = 1
        self.Vent_Sol = 1
        self.Res1_Sol = 0
        self.Res2_Sol = 0
        self.MC_Pump = 0
        self.total_faults = 0
        self.throttle = 0
        self.speed = -1
        self.distance = -1
        self.accel = -1
        self.MET = 0
        self.MET_starttime = -1
        self.stopped_time = -1
        self.para_max_accel = 0  # [G]
        self.para_max_speed = 0  # ft/s
        self.para_max_time = 0  # sec
        self.para_BBP = 0  # ft

        # SPACEX CONFIG DATA
        self.spacex_state = 0
        self.spacex_team_id = 69
        self.spacex_server_ip = '192.168.0.1'
        self.spacex_server_port = 3000
        self.spacex_rate = 40               # Hz
        self.spacex_lastsend = 0

        # DEBUG init for script:
        self.Quit = False

        # Pod Abort conditions init:
        self.Fault = False
        self.Trigger = False

        # INITIATE STATE TO S2A
        self.state = self.SafeToApproach

        #INITIATE LOG RATE INFO
        self.log_lastwrite = clock()            # Saves last time of file write
        self.log_rate = 10                      # Hz

        # Create Abort Range Dictionary
        abort_names = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                       dtype=str)
        abort_vals = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 12),
                                      dtype=float)
        for i in range(0, len(abort_names)):
            if abort_vals[i, 2] == 1:
                self.abort_ranges[self.SafeToApproach][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                    'High': abort_vals[i, 1],
                                                    'Trigger': abort_vals[i, 9],
                                                    'Fault': abort_vals[i, 10]
                                                    }
            if abort_vals[i, 4] == 1:
                self.abort_ranges[self.Launching][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                          'High': abort_vals[i, 1],
                                                          'Trigger': abort_vals[i, 9],
                                                          'Fault': abort_vals[i, 10]
                                                          }
            if abort_vals[i, 5] == 1:
                self.abort_ranges[self.BrakingHigh][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                       'High': abort_vals[i, 1],
                                                       'Trigger': abort_vals[i, 9],
                                                       'Fault': abort_vals[i, 10]
                                                       }
            if abort_vals[i, 7] == 1:
                self.abort_ranges[self.Crawling][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                         'High': abort_vals[i, 1],
                                                         'Trigger': abort_vals[i, 9],
                                                         'Fault': abort_vals[i, 10]
                                                         }
            if abort_vals[i, 8] == 1:
                self.abort_ranges[self.BrakingLow][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                    'High': abort_vals[i, 1],
                                                                    'Trigger': abort_vals[i, 9],
                                                                    'Fault': abort_vals[i, 10]
                                                                    }


        # Create Commands Dictionary
        cmd_names = numpy.genfromtxt('commands.txt', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                       dtype=str)
        cmd_vals = numpy.genfromtxt('commands.txt', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 2),
                                      dtype=int)
        for i in range(0, len(cmd_names)):
            self.commands[cmd_names[i]] = cmd_vals[i]

        ### Create log file ###
        date = datetime.datetime.today()
        new_number = str(date.year) + str(date.month) + str(date.day) \
                     + str(date.hour) + str(date.minute) + str(date.second)
        self.file_name = 'log_' + new_number
        file = open(os.path.join('logs/', self.file_name), 'a')
        columns = [
            'Label',
            'Value',
            'Fault',
            'Time'
        ]
        with file:
            file.write('\t'.join(map(lambda column_title: "\"" + column_title + "\"", columns)))
            file.write("\n")
        file.close()


        print("Pod init complete, State: " + str(self.state))
        print("Log file created: " + str(self.file_name))

    #debug
    sensor_data['Brake_Pressure'] = 178

def poll_sensors():
    ### CAN DATA ###
    PodStatus.sensor_data['BMS_Conn'] = 1                           # Determines if BMS is responding
    PodStatus.sensor_data['BMS_Cell_Temp_Leader'] = max([35])       # Finds maximum cell temp value
    PodStatus.sensor_data['BMS_Cell_Voltage_Leader'] = max([4.2])   # Finds maximum cell voltage value
    PodStatus.sensor_data['BMS_Cell_Voltage_Laggard'] = min([4.2])  # Finds min cell voltage value
    PodStatus.sensor_data['BMS_Pack_Voltage'] = 600                 # Finds total pack voltage (adds all cells up)
    PodStatus.sensor_data['SD_Conn'] = 1
    PodStatus.sensor_data['SD_Temp'] = 30
    PodStatus.sensor_data['SD_HV_Current'] = 0
    PodStatus.sensor_data['Motor_Speed'] = 0
    PodStatus.sensor_data['Motor_Distance'] = 0

    ### I2C DATA ###
    PodStatus.sensor_data['LVBatt_Temp'] = 20
    PodStatus.sensor_data['LVBatt_Current'] = 5
    PodStatus.sensor_data['LVBatt_Voltage'] = 11.7
    PodStatus.sensor_data['PV_Left_Temp'] = 30
    PodStatus.sensor_data['PV_Left_Pressure'] = 12
    PodStatus.sensor_data['PV_Right_Temp'] = 30
    PodStatus.sensor_data['PV_Right_Pressure'] = 12
    PodStatus.sensor_data['Ambient_Pressure'] = 0.1
    PodStatus.sensor_data['IMU1_X'] = 0
    PodStatus.sensor_data['IMU1_Y'] = -1.02
    PodStatus.sensor_data['IMU1_Z'] = 0
    PodStatus.sensor_data['IMU2_X'] = 0
    PodStatus.sensor_data['IMU2_Y'] = -1.02
    PodStatus.sensor_data['IMU2_Z'] = 0
    PodStatus.sensor_data['LIDAR'] = 0
    #PodStatus.sensor_data['Brake_Pressure'] = 178
    PodStatus.sensor_data['LST_Left'] = 0
    PodStatus.sensor_data['LST_Right'] = 0

    ### RPi DATA ###
    PodStatus.sensor_data['GUI_Conn'] = 1
    PodStatus.sensor_data['RPi_Temp'] = 40
    PodStatus.sensor_data['RPi_Proc_Load'] = 5
    PodStatus.sensor_data['RPi_Mem_Load'] = 5
    PodStatus.sensor_data['RPi_Disk_Space'] = 14000

    ### SPACEX DATA ###
    PodStatus.stripe_count = numpy.mean([PodStatus.sensor_data['LST_Left'], PodStatus.sensor_data['LST_Right']])

    if PodStatus.sensor_data['Brake_Pressure'] > 177:
        PodStatus.Brakes = False
    else:
        PodStatus.Brakes = True

    PodStatus.speed = PodStatus.sensor_data['Motor_Speed']
    PodStatus.accel = PodStatus.sensor_data['IMU1_X']
    #PodStatus.distance = PodStatus.sensor_data['Motor_Distance']

    if PodStatus.MET > 0:
        PodStatus.MET = clock()-PodStatus.MET_starttime

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
        #   8 - Crawling bool
        #   9 - Brake2 bool
        #   10 - Trigger bool
        #   11 - Fault bool

    if PodStatus.Fault == True:
        abort()

    #a = numpy.shape(PodStatus.abort_ranges) # Load abort_ranges size into tuple, so we can iterate any array size

    if PodStatus.state == PodStatus.SafeToApproach:   # Evaluates abort criteria for Safe To Approach state
        for key in PodStatus.abort_ranges[PodStatus.SafeToApproach]:
            if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Low'] \
                    or PodStatus.sensor_data[key] > PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['High']:
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
                print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Low']) +
                      " to " + str(PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['High']))
                PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Fault'] = 1
                if PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Trigger'] == 1:
                    print("Aborting for " + str(PodStatus.sensor_data[str(key)]))
                    abort()
                    break

        PodStatus.total_faults = 0
        for key in PodStatus.abort_ranges[PodStatus.SafeToApproach]:
            if PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Fault'] == 1:
                PodStatus.total_faults += 1
        if PodStatus.total_faults > 0:
            print("Faults: \n" + str(PodStatus.abort_ranges[PodStatus.SafeToApproach][str(key)]['Fault']))
        else:
            PodStatus.Fault = False

    if PodStatus.state == PodStatus.Launching:   # Evaluates abort criteria for Safe To Approach state
        for key in PodStatus.abort_ranges[PodStatus.Launching]:
            if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.Launching][str(key)]['Low'] \
                    or PodStatus.sensor_data[key] > PodStatus.abort_ranges[PodStatus.Launching][str(key)]['High']:
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
                print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.Launching][str(key)]['Low']) + " to " + str(PodStatus.abort_ranges[PodStatus.Launching][str(key)]['High']))
                PodStatus.total_faults += 1
                if PodStatus.abort_ranges[PodStatus.Launching][str(key)]['Trigger'] == 1:
                    print("Aborting for " + str(PodStatus.sensor_data[str(key)]))
                    abort()
                    break

        PodStatus.total_faults = 0
        for key in PodStatus.abort_ranges[PodStatus.Launching]:
            if PodStatus.abort_ranges[PodStatus.Launching][str(key)]['Fault'] == 1:
                PodStatus.total_faults += 1
        if PodStatus.total_faults > 0:
            print("Faults: \n" + str(PodStatus.abort_ranges[PodStatus.Launching][str(key)]['Fault']))
        else:
            PodStatus.Fault = False

    if PodStatus.state == PodStatus.BrakingHigh:   # Evaluates abort criteria for Safe To Approach state
        for key in PodStatus.abort_ranges[PodStatus.BrakingHigh]:
            if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.BrakingHigh][str(key)]['Low'] \
                    or PodStatus.sensor_data[key] > PodStatus.abort_ranges[PodStatus.BrakingHigh][str(key)]['High']:
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
                print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.BrakingHigh][str(key)]['Low']) + " to "
                      + str(PodStatus.abort_ranges[PodStatus.BrakingHigh][str(key)]['High']))
                PodStatus.total_faults += 1
                if PodStatus.abort_ranges[PodStatus.BrakingHigh][str(key)]['Trigger'] == 1:
                    abort()

    if PodStatus.state == PodStatus.Crawling:   # Evaluates abort criteria for Safe To Approach state
        for key in PodStatus.abort_ranges[PodStatus.Crawling]:
            if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.Crawling][str(key)]['Low'] \
                    or PodStatus.sensor_data[key] > PodStatus.abort_ranges[PodStatus.Crawling][str(key)]['High']:
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
                print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.Crawling][str(key)]['Low']) + " to "
                      + str(PodStatus.abort_ranges[PodStatus.Crawling][str(key)]['High']))
                PodStatus.total_faults += 1
                if PodStatus.abort_ranges[PodStatus.Crawling][str(key)]['Trigger'] == 1:
                    abort()

    #print("Total Faults: " + str(PodStatus.total_faults))
    if PodStatus.total_faults > 0: PodStatus.Fault = True

    #     # DEPRECATED
    #     # for i in range(a[0]):
    #     #     temp_range = PodStatus.abort_ranges[i]    # Loads current abort range
    #     #     temp_sensor = PodStatus.sensor_data[i,:]    # Loads current sensor data
    #     #     if temp_range[3] == 1:                    # Evaluates the S2A column value
    #     #         if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
    #     #             PodStatus.abort_ranges[i,11] = 1
    #     #             # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #     #             print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #     #                   "\tValue: " + str(temp_sensor[1]) +
    #     #                   "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #     #             if temp_range[10] == 1:    # IF FAULT IS A TRIGGER, ABORT()
    #     #                 abort()
    #     #         elif temp_sensor[1] > temp_range[2]:    # Evaluates "HIGH" values
    #     #             PodStatus.abort_ranges[i,11] = 1
    #     #             # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #     #             print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #     #                   "\tValue: " + str(temp_sensor[1]) +
    #     #                   "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #     #             if temp_range[10] == 1:    # IF FAULT IS A TRIGGER, ABORT()
    #     #                 abort()
    #     #         else:
    #     #             PodStatus.abort_ranges[i,11] = 0
    #
    # if PodStatus.state == PodStatus.Launching:        # Evaluates abort criteria for FC2L state
    #     for i in range(a[0]):
    #         temp_range = PodStatus.abort_ranges[i]    # Loads current sensor range to temp
    #         temp_sensor = PodStatus.sensor_data[i,:]
    #         if temp_range[5] == 1:                    # Evaluates the FC2L column value
    #             if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             elif (temp_sensor[1] > temp_range[2]):    # Evaluates "HIGH" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             else:
    #                 PodStatus.abort_ranges[i,11] = 0
    #
    # if PodStatus.state == PodStatus.BrakingHigh:   # Evaluates abort criteria for Safe To Approach state
    #     for i in range(a[0]):
    #         temp_range = PodStatus.abort_ranges[i]    # Loads current abort range
    #         temp_sensor = PodStatus.sensor_data[i,:]    # Loads current sensor data
    #         if temp_range[6] == 1:                    # Evaluates the S2A column value
    #             if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             elif temp_sensor[1] > temp_range[2]:    # Evaluates "HIGH" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             else:
    #                 PodStatus.abort_ranges[i,11] = 0
    #
    # if PodStatus.state == PodStatus.Crawling:        # Evaluates abort criteria for FC2L state
    #     for i in range(a[0]):
    #         temp_range = PodStatus.abort_ranges[i]    # Loads current sensor range to temp
    #         temp_sensor = PodStatus.sensor_data[i,:]
    #         if temp_range[8] == 1:                    # Evaluates the FC2L column value
    #             if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             elif (temp_sensor[1] > temp_range[2]):    # Evaluates "HIGH" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #             else:
    #                 PodStatus.abort_ranges[i,11] = 0
    #
    # if PodStatus.state == PodStatus.BrakingLow:   # Evaluates abort criteria for Safe To Approach state
    #     for i in range(a[0]):
    #         temp_range = PodStatus.abort_ranges[i]    # Loads current abort range
    #         temp_sensor = PodStatus.sensor_data[i,:]    # Loads current sensor data
    #         if temp_range[9] == 1:                    # Evaluates the S2A column value
    #             if temp_sensor[1] < temp_range[1]:    # Evaluates "LOW" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #                 if temp_sensor[10] == 1:    # IF FAULT IS A TRIGGER, ABORT()
    #                     abort()
    #             elif temp_sensor[1] > temp_range[2]:    # Evaluates "HIGH" values
    #                 PodStatus.abort_ranges[i,11] = 1
    #                 # NEED "STORE FAULTS" FUNCTION HERE TO RECORD TIME OF FAULT
    #                 print("Pod Fault!\tSensor: " + str(PodStatus.abort_labels[i]) +
    #                       "\tValue: " + str(temp_sensor[1]) +
    #                       "\t Range: "+ str(temp_range[1]) + " to " + str(temp_range[2]))
    #                 if temp_sensor[10] == 1:    # IF FAULT IS A TRIGGER, ABORT()
    #                     abort()
    #             else:
    #                 PodStatus.abort_ranges[i,11] = 0


def rec_data():     # This function parses received data into useable commands by SDA.

    ### TEST SCRIPT FOR FAKE COMMAND DATA / GUI
    print("\n******* POD STATUS ******\n"
        "* State:         " + str(PodStatus.state) + "\t\t*")
    print("* Pod Clock Time: " + str(round(PodStatus.MET,3)) + "\t*")
    print("* Fault:         " + str(PodStatus.Fault) + "\t\t*")
    print("* HV System:     " + str(PodStatus.HV) + "\t\t*")
    print("* Brakes:        " + str(PodStatus.Brakes) + "\t*\n"
        "* Vent Solenoid: " + str(PodStatus.Vent_Sol) + "\t*\n"
        "* Res1 Solenoid: " + str(PodStatus.Res1_Sol) + "\t*\n"
        "* Res2 Solenoid: " + str(PodStatus.Res2_Sol) + "\t*\n"
        "* MC Pump:       " + str(PodStatus.MC_Pump) + "\t*\n"
        "* Flight BBP:       " + str(PodStatus.para_BBP) + "\t*\n"
        "* Flight Speed:       " + str(PodStatus.para_max_speed) + "\t*\n"
        "* Flight Accel:       " + str(PodStatus.para_max_accel) + "\t*\n"
        "* Flight Time:       " + str(PodStatus.para_max_time) + "\t*\n"
        "*************************")

    if PodStatus.state == PodStatus.SafeToApproach:
        print("\n*** MENU ***\n\t1. Launch\n\t2. HV On/Off\n\t3. Vent Solenoid Open/Close"
              "\n\t4. Brake Res #1 Open/Close\n\t5. Brake Res #2 Open/Close"
              "\n\t6. MC Coolant Pump On/Off\n\t7. Quit\n\t8. Set BBP\n\t9. Set Speed\n\t10. Set Accel\n\t11. Set Time\n\t"
              "12. Reset Abort Flag\n\t")
        a = input('Enter choice: ')
        if a == '1':
            PodStatus.commands['Launch'] = 1
        elif a == '2':
            if PodStatus.HV == 0:
                PodStatus.commands['HV'] = 1
                PodStatus.HV = 1
            else:
                PodStatus.commands['HV'] = 0
                PodStatus.HV = 0
        elif a == '3':
            if PodStatus.commands['Vent_Sol'] == 0:
                PodStatus.commands['Vent_Sol'] = 1           # Brake Vent opens
                PodStatus.Vent_Sol = 0
                PodStatus.sensor_data['Brake_Pressure'] = 15      # Change brake pressure to atmo
            else:
                PodStatus.commands['Vent_Sol'] = 0
                PodStatus.Vent_Sol = 1
        elif a == '4':
            if PodStatus.commands['Res1_Sol'] == 0:
                PodStatus.commands['Res1_Sol'] = 1
                if PodStatus.Vent_Sol == 1:
                    PodStatus.sensor_data['Brake_Pressure'] = 200
            else:
                PodStatus.commands['Res1_Sol'] = 0
        elif a == '5':
            if PodStatus.commands['Res2_Sol'] == 1:
                PodStatus.commands['Res2_Sol'] = 0
            else:
                PodStatus.commands['Res2_Sol'] = 1
        elif a == '6':
            if PodStatus.commands['MC_Pump'] == 0:
                PodStatus.commands['MC_Pump'] = 1
                PodStatus.MC_Pump = 1
            else:
                PodStatus.commands['MC_Pump'] = 0
                PodStatus.MC_Pump = 0
        elif a == '7':
            PodStatus.Quit = True
        elif a == '8':
            PodStatus.para_BBP = float(input("Enter BBP Distance in feet"))
        elif a == '9':
            PodStatus.para_max_speed = float(input("Enter max speed in ft/s"))
        elif a == '10':
            PodStatus.para_max_accel = float(input("Enter max accel in G"))
        elif a == '11':
            PodStatus.para_max_time = float(input("Enter max time in s"))
        elif a == '12':
            PodStatus.commands['Abort'] = 0
        else:
            pass

    elif PodStatus.state == PodStatus.Launching:
        print("\n*** MENU ***\n\t1. Abort\n\t2. Brake\n\t3. Quit")
        a = input('Enter choice: ')
        if a == '1':
            PodStatus.commands['Abort'] = 1
            abort()
        elif a == '2':
            PodStatus.distance = PodStatus.para_BBP + 1
            print(PodStatus.distance)
        elif a == '3':
            PodStatus.Quit = True
        else:
            pass
    elif PodStatus.state == PodStatus.BrakingHigh:
        pass
    elif PodStatus.state == PodStatus.Crawling:
        print("\n*** MENU ***\n\t1. Abort\n\t2. Quit")
        a = input('Enter choice: ')
        if a == '1':
            PodStatus.commands['Abort'] = 1
            abort()
        elif a == '2':
            PodStatus.Quit = True
        else:
            pass
    elif PodStatus.state == PodStatus.BrakingLow:
        pass
    else:
        pass
    ### END TEST SCRIPT


def do_commands():
    if PodStatus.state == 1:    # Load ALL commands for full GUI control
        if PodStatus.commands['Abort'] == 1:
            PodStatus.Fault = True
            print("*** ABORT COMMAND RECEIVED ***")
            abort()

        # Launch pod if conditions in run_state() for spacex_state are met
        if PodStatus.commands['Launch'] == 1 and PodStatus.spacex_state == 2:
            transition()
        elif PodStatus.commands['Launch'] == 1 and PodStatus.spacex_state != 2:
            print("Pod not configured for launch, resetting Launch command to 0.")
            PodStatus.commands['Launch'] = 0

        # Turn on/off HV contactors as appropriate
        if PodStatus.commands['HV'] != PodStatus.HV:
            if PodStatus.HV == 0:
                PodStatus.HV = 1
            else:
                PodStatus.HV = 0

        # Change Vent Solenoid
        if PodStatus.commands['Vent_Sol'] != PodStatus.Vent_Sol:
            if PodStatus.Vent_Sol == 0:
                PodStatus.Vent_Sol = 1
            else:
                PodStatus.Vent_Sol = 0
                PodStatus.sensor_data['Brake_Pressure'] = 0

        # Change Reservoir #1 Solenoid
        if PodStatus.commands['Res1_Sol'] != PodStatus.Res1_Sol:  # If command does not equal current state
            if PodStatus.Res1_Sol == 0:
                ### SET RESERVOIR SOLENOID TO CLOSE
                ## DEBUG:
                PodStatus.Res1_Sol = 1
            else:
                ### SET RESERVOIR SOLENOID TO OPEN
                ## DEBUG:
                PodStatus.Res1_Sol = 0

        # Change Reservoir #2 Solenoid
        if PodStatus.commands['Res2_Sol'] != PodStatus.Res2_Sol:
            if PodStatus.Res2_Sol == 0:
                PodStatus.Res2_Sol = 1
            else:
                PodStatus.Res2_Sol = 0

        # Change Coolant Pump state
        if PodStatus.commands['MC_Pump'] != PodStatus.MC_Pump:
            if PodStatus.MC_Pump == 0:
                PodStatus.MC_Pump = 1
            else:
                PodStatus.MC_Pump = 0

    else:
        # Load ONLY abort command
        if PodStatus.commands['Abort'] == True:
            abort()



def spacex_data():
    if (clock()-PodStatus.spacex_lastsend) < (1/PodStatus.spacex_rate):
        #print("No packet sent.")
        pass
    else:
        PodStatus.spacex_lastsend = clock()
        #print("SpaceX packet sent at " + str(PodStatus.spacex_lastsend))

        ### SpaceX-provided code
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = (PodStatus.spacex_server_ip, PodStatus.spacex_server_port)
        packet = struct.pack(">BB7iI", PodStatus.spacex_team_id, PodStatus.spacex_state, int(PodStatus.accel),
                             int(PodStatus.distance), int(PodStatus.speed), 0, 0, 0, 0, int(PodStatus.stripe_count) // 3048)
        sock.sendto(packet, server)



def send_data():        # Sends data to UDP (GUI) and CAN (BMS/MC)

    ### Send to CAN ###

    ### Send to UDP ###

    pass

def run_state():

    # S2A STATE
    if PodStatus.state == 1:

        # Determine if SpaceX state = 1 (S2A) or 2 (Ready to Launch)
        if (
            PodStatus.Fault == False and
            PodStatus.HV == True and
            PodStatus.Brakes == False and
            PodStatus.para_BBP > 0 and
            PodStatus.para_max_accel > 0 and
            PodStatus.para_max_speed > 0 and
            PodStatus.para_max_time > 0):
            PodStatus.spacex_state = 2
            print("Pod is Ready for Launch (SpaceX State 2)")
        else:
            PodStatus.spacex_state = 1

        # TRANSITIONS
        # None.  Only transition from S2A comes from do_commands() function


    # LAUNCHING STATE
    elif PodStatus.state == 3:
        PodStatus.spacex_state = 3
        PodStatus.commands['Launch'] = 0        # Resets launch command once successfully launched

        # Start the flight clock
        if PodStatus.MET_starttime == -1:
            print("The clock has started.")
            PodStatus.MET_starttime = clock()
        else:
            PodStatus.MET = clock()-PodStatus.MET_starttime

        # ACCEL UP TO MAX G within 2%
        if PodStatus.sensor_data['IMU1_X'] < (0.98 * PodStatus.para_max_accel):
            PodStatus.throttle = PodStatus.throttle * 1.01
        elif PodStatus.sensor_data['IMU1_X'] > (1.02*PodStatus.para_max_accel):
            PodStatus.throttle = PodStatus.throttle * 0.99

        # TRANSITIONS
        if PodStatus.distance > PodStatus.para_BBP:
            print("Pod has crossed BBP.")
            transition()
        if PodStatus.speed > PodStatus.para_max_speed:
            print("Pod has reached max speed.")
            transition()
        if PodStatus.MET > PodStatus.para_max_time:
            print("Pod has exceeded max time.")
            transition()


    # COAST (NOT USED)
    elif PodStatus.state == 4:
        PodStatus.spacex_state = 4
        pass

    # BRAKE, HIGH SPEED
    elif PodStatus.state == 5:
        PodStatus.MET = clock()-PodStatus.MET_starttime
        PodStatus.spacex_state = 5

        # if PodStatus.speed > 0.5:
        #     PodStatus.stopped_time = 0
        if PodStatus.speed > 0.5:
            if PodStatus.commands['Vent_Sol'] == 1:    # OPEN BRAKE VENT SOLENOID
                print("Opening Vent Sol")
                PodStatus.commands['Vent_Sol'] = 0
            if PodStatus.throttle > 0:          # SET THROTTLE TO 0
                PodStatus.throttle = 0

        # DO NOTHING ELSE UNTIL STOPPED

        # RECONFIGURE FOR CRAWLING
        if PodStatus.speed < 0.5:
            if PodStatus.stopped_time == -1:
                timer = 0
                PodStatus.stopped_time = clock()
                PodStatus.commands['Vent_Sol'] = 0
                print("Stopped at " + str(round(PodStatus.stopped_time, 2)) + " seconds; Holding for 2 seconds.")
            else:
                timer = clock() - PodStatus.stopped_time

            if timer > 2:
                if PodStatus.commands['Vent_Sol'] == 0:
                    PodStatus.sensor_data['Brake_Pressure'] = 0
                    PodStatus.commands['Vent_Sol'] = 1     # CLOSE BRAKE VENT SOLENOID
                    print("Closing Vent Sol")
                if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] < 20:
                    PodStatus.commands['Res1_Sol'] = 1     # OPEN RES#1 SOLENOID
                    print("Opening Res#1, pausing for 2 seconds.")
                    sleep(2)
                    PodStatus.commands['Res1_Sol'] = 0     # CLOSE RES#1 SOLENOID
                    transition()


    # CRAWLING
    elif PodStatus.state == 6:
        PodStatus.spacex_state = 6
        pass
        # DO STUFF

    # BRAKE, FINAL
    elif PodStatus.state == 7:
        PodStatus.spacex_state = 5
        print("Braking, Final")
        if PodStatus.speed < 1:
            print("Pod stopped")
            transition()
        # DO STUFF

    else:
        print("Invalid pod state found: " + str(PodStatus.state))
        print("Quitting")
        PodStatus.Fault = True
        PodStatus.Quit = True


def transition():
    if PodStatus.state == 1:          # S2A trans
        PodStatus.state = 3
        print("TRANS: S2A(1) to LAUNCH(3)")

    elif PodStatus.state == 3:          # LAUNCH trans
        PodStatus.state = 5
        print("TRANS: LAUNCH(3) TO BRAKE(5)")

    elif PodStatus.state == 5:
        PodStatus.state = 6
        print("TRANS: BRAKE(5) to CRAWLING(6)")

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
        PodStatus.Quit = True

def abort():
    if PodStatus.state == 1:          # S2A STATE FUNCTIONS
        print("Abort attempted in S2A")

    elif PodStatus.state == 3:          # LAUNCH STATE FUNCTIONS
        print("ABORTING from 3 to 7")
        PodStatus.state = 7

    elif PodStatus.state == 5:
        print("ABORTING from 5 to 7")
        PodStatus.state = 7

    elif PodStatus.state == 6:
        PodStatus.state = 7
    elif PodStatus.state == 7:
        if PodStatus.speed > 0:
            print("Waiting for pod to stop.")
        else:
            PodStatus.state = 1
    else:
        PodStatus.state = 1

def write_file():
    # SLOW THIS THE FUCK DOWN
    if (clock() - PodStatus.log_lastwrite) < (1/PodStatus.log_rate):
        pass
    else:
        file = open(os.path.join('logs/', PodStatus.file_name), 'a')
        with file:
            for key in PodStatus.sensor_data:
                if str(key) in PodStatus.abort_ranges[PodStatus.state]:
                    fault_code = PodStatus.abort_ranges[PodStatus.state][str(key)]['Fault']
                else:
                    fault_code = 0

                line = str(key) + '\t' + str(PodStatus.sensor_data[str(key)]) + '\t' \
                       + str(int(fault_code)) + '\t' + str(round(clock(),2)) + '\n'
                file.write(line)
            for key in PodStatus.commands:
                line = str(key) + '\t' + str(PodStatus.commands[str(key)]) + '\t\t' + str(round(clock(),2)) + '\n'
                file.write(line)
            line = 'Pod State' + '\t' + str(PodStatus.state) + '\t\t' + str(round(clock(),2)) + '\n'
            file.write(line)
            line = 'SpaceX state' + '\t' + str(PodStatus.spacex_state) + '\t\t' + str(round(clock(),2)) + '\n'
            file.write(line)
        PodStatus.log_lastwrite = clock()


if __name__ == "__main__":

    PodStatus = Status()


    while PodStatus.Quit == False:
        write_file()
        poll_sensors()
        run_state()
        do_commands()
        eval_abort()
        rec_data()
        send_data()
        spacex_data()
        #print(clock())

    # DEBUG...REMOVE BEFORE FLIGHT
    print("Quitting")

