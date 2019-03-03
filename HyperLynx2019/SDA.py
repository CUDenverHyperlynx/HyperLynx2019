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
    # Definition of State Numbers
    SafeToApproach = 1
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7

    # Init dictionaries
    abort_ranges = {}           # Dict of dicts (below)
    abort_ranges[SafeToApproach] = {}
    abort_ranges[Launching] = {}
    abort_ranges[BrakingHigh] = {}
    abort_ranges[Crawling] = {}
    abort_ranges[BrakingLow] = {}
    commands = {}               # Contains all possible inputs from GUI
    sensor_data = {}            # Contains all inputs from I2C/CAN buses

    poll_oldtime = 0    # Vars for poll_sensors for integrating distance from speed
    poll_newtime = 0
    poll_interval = 0

    def __init__(self):        # BOOT INIT

        self.wheel_diameter = 17.4 / 12 # [ft] define drive wheel diameter
        self.StartTime = clock()
        self.HV = 0                     # Current state of HV system (1 or 0)
        self.Brakes = 1                 # Current state of brakes (1 = >177psi, 0 = <177psi)
        self.Vent_Sol = 1               # state of vent solenoid (1 = closed, 0 = open)
        self.Res1_Sol = 0               # state of reservoir #1 solenoid (1 = open, 0 = closed)
        self.Res2_Sol = 0               # state of reservoir #2 solenoid (1 = open, 0 = closed)
        self.MC_Pump = 0                # state of coolant pump (1 = on, 0 = off)
        self.total_faults = 0           # total number of active faults detected
        self.throttle = 0               # current throttle setting to send to SD100 controller
        self.speed = -1                 # [ft/s] init pod speed
        self.distance = -1              # [ft] init pod distance traveled
        self.accel = -1                 # [g] init pod acceleration
        self.stripe_count = 0           # total number of stripes counted
        self.MET = 0                    # Mission Elapsed Time (since launch)
        self.MET_starttime = -1
        self.stopped_time = -1          # Time since coming to a stop
        self.para_max_accel = 0         # [g] Maximum pod acceleration for control loop
        self.para_max_speed = 0         # [ft/s] Maximum pod speed for braking point
        self.para_max_time = 0          # [s] Maximum time of flight for braking point
        self.para_BBP = 0               # [ft] Maximum distance traveled for braking point
        self.para_max_crawl_speed = -1  # [ft/s] Maximum crawling speed.  Init to -1 to allow
                                        # crew to set 0 speed for crawling state

        # SPACEX CONFIG DATA
        self.spacex_state = 0
        self.spacex_team_id = 69
        self.spacex_server_ip = '192.168.0.1'
        self.spacex_server_port = 3000
        self.spacex_rate = 40               # [Hz] rate of spacex data burst
        self.spacex_lastsend = 0

        # DEBUG init for script:
        self.Quit = False

        # Pod Abort conditions init:
        self.Fault = False
        self.Abort = False

        # INITIATE STATE TO S2A
        self.state = self.SafeToApproach

        #INITIATE LOG RATE INFO
        self.log_lastwrite = clock()            # Saves last time of file write to control log rate
        self.log_rate = 10                      # Hz

        # Create Abort Range Dictionary from template file
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


        # Create Commands Dictionary from template file
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

        ## Confirm boot info ##
        print("Pod init complete, State: " + str(self.state))
        print("Log file created: " + str(self.file_name))

    #debug
    sensor_data['Brake_Pressure'] = 178

def poll_sensors():
    """
    Runs all polling functions for the I2C & CAN buses.
    Converts raw data to pod state variables
    Q: Should this be in a different thread?
    """

    PodStatus.poll_oldtime = PodStatus.poll_newtime
    PodStatus.poll_newtime = clock()
    PodStatus.poll_interval = PodStatus.poll_newtime-PodStatus.poll_oldtime

    ### CAN DATA ###
    PodStatus.sensor_data['SD_MotorData_MotorRPM'] = 0


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
    # Sets the highest of the two LST data sets to the pod state variable (most conservative)
    PodStatus.stripe_count = numpy.maximum(PodStatus.sensor_data['LST_Left'], PodStatus.sensor_data['LST_Right'])

    ### CONVERT DATA ###
    # Set pod state variable for brakes
    if PodStatus.sensor_data['Brake_Pressure'] > 177:
        PodStatus.Brakes = False
    else:
        PodStatus.Brakes = True

    # Set pod state variable for speed and acceleration
    old_speed = PodStatus.speed
    PodStatus.speed = PodStatus.sensor_data['SD_MotorData_MotorRPM'] / 60 * 2*numpy.pi*PodStatus.wheel_diameter
    PodStatus.accel = numpy.mean([PodStatus.sensor_data['IMU1_X'], PodStatus.sensor_data['IMU2_X']])

    # Integrate distance.  At 1.4GHz clock speeds, integration can be numerically
    # approximated as constant speed over the time step.
    if PodStatus.poll_oldtime != 0:         # Update distance (but skip init loop @ old_time = 0
        PodStatus.distance += PodStatus.speed * PodStatus.poll_interval

    # Update MET
    if PodStatus.MET > 0:
        PodStatus.MET = clock()-PodStatus.MET_starttime

def eval_abort():
    """
    Determines if any received commands or sensor data places the pod into an abort condition.
    Abort conditions are unique to each state- what may cause an abort in the Launching state may not
    cause an abort in the Braking states.  Each state has a unique dictionary of abort conditions,
    all contained in the abort_ranges template file.

    This data is stored in a dict for easy access.

    For Example:  to find the highest allowable HV battery cell temperature during the Launching state,
     you would query:
    PodStatus.sensor_data[PodStatus.Launching]['BMS_HighestTemp']['High']
     To find the actual value, you would simply query:
    PodStatus.sensor_data[PodStatus.Launching]['BMS_HighestTemp']

    If the 'Trigger' value is flagged as a '1' value, this means the abort condition would trigger
    an abort for the pod.  There are many data points which may cause a fault, but not cause a direct
    internal pod abort.  These data points are left to the crew to adjudicate, and are generally
    non-critical.

    """

    for key in PodStatus.abort_ranges[PodStatus.state]:     # Search abort_ranges dict for current state
        # if out of range, log 'Fault' key as 1
        if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.state][str(key)]['Low'] \
                or PodStatus.sensor_data[key] > PodStatus.abort_ranges[PodStatus.state][str(key)]['High']:
            ### DEBUG PRINT
            print("Pod Fault!\tSensor: " + str(key))
            print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
            print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.state][str(key)]['Low']) +
                  " to " + str(PodStatus.abort_ranges[PodStatus.state][str(key)]['High']))
            PodStatus.abort_ranges[PodStatus.state][str(key)]['Fault'] = 1

    PodStatus.total_triggers = 0
    PodStatus.total_faults = 0      # Reset total_fault count to 0 each loop
    # Sum total number of faults for this state
    for key in PodStatus.abort_ranges[PodStatus.state]:
        if PodStatus.abort_ranges[PodStatus.state][str(key)]['Fault'] == 1:
            PodStatus.total_faults += 1
            if PodStatus.abort_ranges[PodStatus.state][str(key)]['Trigger'] == 1:
                PodStatus.total_triggers += 1

    if PodStatus.total_faults > 0:
        PodStatus.Fault = True
        ### DEBUG PRINT
        print("Number of Faults: \n" + str(int(PodStatus.total_faults)))
    else:
        PodStatus.Fault = False

    if PodStatus.total_triggers > 0:
        ### DEBUG PRINT TRIGGERS
        print("ABORT TRIGGERS FOUND: \n" + str(int(PodStatus.total_triggers)))
        print("FLAGGING ABORT == TRUE")
        PodStatus.Abort = True         # This is the ONLY location an abort can be reached during this function

    if PodStatus.Abort == True:
        abort()

def rec_data():
    """
    This function receives data from the GUI, primarily commands[]
    During DEBUG, this function is a mock GUI for testing SDA.
    """

    ### TEST SCRIPT FOR FAKE COMMAND DATA / GUI
    print("\n******* POD STATUS ******\n"
        "\tState:               " + str(PodStatus.state) + "\t\t")
    print("\tPod Clock Time:    " + str(round(PodStatus.MET,3)) + "\t")
    print("\tFault:             " + str(PodStatus.Fault) + "\t\t")
    print("\tHV System:         " + str(PodStatus.HV) + "\t\t")
    print("\tBrakes:            " + str(PodStatus.Brakes) + "\t\n"
        "\tVent Solenoid:       " + str(PodStatus.Vent_Sol) + "\t\n"
        "\tRes1 Solenoid:       " + str(PodStatus.Res1_Sol) + "\t\n"
        "\tRes2 Solenoid:       " + str(PodStatus.Res2_Sol) + "\t\n"
        "\tMC Pump:             " + str(PodStatus.MC_Pump) + "\t\n"
        "\tFlight BBP:          " + str(PodStatus.para_BBP) + "\t\n"
        "\tFlight Speed:        " + str(PodStatus.para_max_speed) + "\t\n"
        "\tFlight Accel:        " + str(PodStatus.para_max_accel) + "\t\n"
        "\tFlight Time:         " + str(PodStatus.para_max_time) + "\t\n"
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
    """
    This function ensures all current commands[] are run each loop.  The GUI will show the last
    sent command as well as the current state of the commands[] dict to ensure proper operation.

    During the S2A state, all commands are available to the crew through the GUI.  This allows
    easy testing and preflight.  All flight parameters are received through this function.

    This is the ONLY function that will allow the pod to transition from S2A to Launching.

    During flight, the ONLY command that the SDA will listen to is the Abort signal.

    If an abort takes place, the pod will eventually arrive back to the S2A state, giving full
    control back to the crew.
    """

    if PodStatus.state == 1:    # Load ALL commands for full GUI control
        if PodStatus.commands['Abort'] == 1:
            PodStatus.Abort = True
            print("*** ABORT COMMAND RECEIVED ***")
            abort()

        # Allows crew to reset Abort flag.  ONLY way to reset Abort flag.  Cannot be changed in-flight.
        elif PodStatus.commands['Abort'] == 0 and PodStatus.Abort == True:
            PodStatus.Abort = False
            print("Resetting Abort flag to False")

        ### POD WILL LAUNCH WITH THIS SECTION ###
        # Launch pod ONLY if conditions in run_state() for spacex_state are met
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

        # Load parameters
        if PodStatus.commands['para_BBP'] != PodStatus.para_BBP:
            PodStatus.para_BBP = PodStatus.commands['para_BBP']
        if PodStatus.commands['para_max_speed'] != PodStatus.para_max_speed:
            PodStatus.para_max_speed = PodStatus.commands['para_max_speed']
        if PodStatus.commands['para_max_accel'] != PodStatus.para_max_accel:
            PodStatus.para_max_accel = PodStatus.commands['para_max_accel']
        if PodStatus.commands['para_max_time'] != PodStatus.para_max_time:
            PodStatus.para_max_time = PodStatus.commands['para_max_time']

    else:
        # Load ONLY abort command
        if PodStatus.commands['Abort'] == True:
            abort()

def spacex_data():
    """
    This function passes the required SpaceX data packet at the defined rate.
    """
    ### CONVERT DATA TO SPACEX SPECIFIED UNIT
    accel = PodStatus.accel * 3217.4        # g (unitless) to cm/s2
    speed = PodStatus.speed * 30.48         # ft/s to cm/s
    distance = PodStatus.distance * 30.48   # ft to cm
    if (clock()-PodStatus.spacex_lastsend) < (1/PodStatus.spacex_rate):
        #print("No packet sent.")
        pass
    else:
        PodStatus.spacex_lastsend = clock()
        #print("SpaceX packet sent at " + str(PodStatus.spacex_lastsend))

        ### SpaceX-provided code
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = (PodStatus.spacex_server_ip, PodStatus.spacex_server_port)
        packet = struct.pack(">BB7iI", PodStatus.spacex_team_id, PodStatus.spacex_state, int(accel),
                             int(distance), int(speed), 0, 0, 0, 0, int(PodStatus.stripe_count) // 3048)
        sock.sendto(packet, server)

def send_data():        # Sends data to UDP (GUI) and CAN (BMS/MC)

    ### Send to CAN ###

    ### Send to UDP ###

    pass

def run_state():
    """
    This is the primary "run" function for each state.
    """

    # S2A STATE
    if PodStatus.state == 1:
        # Determine if SpaceX state = 1 (S2A) or 2 (Ready to Launch)
        if (PodStatus.Fault == False and
            PodStatus.Abort == False and
            PodStatus.HV == True and
            PodStatus.Brakes == False and
            PodStatus.para_BBP > 0 and
            PodStatus.para_max_accel > 0 and
            PodStatus.para_max_speed > 0 and
            PodStatus.para_max_time > 0 and
            PodStatus.para_max_crawl_speed > -1):
                PodStatus.spacex_state = 2
                print("Pod is Ready for Launch (SpaceX State 2)")
        else:
            PodStatus.spacex_state = 1

        # TRANSITIONS
        # None.  Only transition from S2A comes from do_commands() function


    # LAUNCHING STATE
    elif PodStatus.state == 3:
        PodStatus.spacex_state = 3
        # print("Resetting Launch command to 0")
        PodStatus.commands['Launch'] = 0        # Resets launch command once successfully launched

        # Start the flight clock
        if PodStatus.MET_starttime == -1:
            print("The MET clock has started.")
            PodStatus.MET_starttime = clock()
        else:
            PodStatus.MET = clock()-PodStatus.MET_starttime

        # ACCEL UP TO MAX G within 2%
        # Linear inputs; MC has a built-in throttle damper
        if PodStatus.accel < (0.98 * PodStatus.para_max_accel):
            PodStatus.throttle = PodStatus.throttle * 1.01
        elif PodStatus.accel > (1.02*PodStatus.para_max_accel):
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

        # GET OUT OF THIS STATE
        PodStatus.state = PodStatus.BrakingLow
        PodStatus.Abort = True
        pass

    # BRAKE, HIGH SPEED
    elif PodStatus.state == 5:
        PodStatus.MET = clock()-PodStatus.MET_starttime
        PodStatus.spacex_state = 5

        if PodStatus.speed > 0.5:       # THIS VALUE NEEDS TO BE THOROUGHLY TESTED;
                                        # IF ERRANT SPEED VALUES > 0.5 WHILE ACTUALLY
                                        # STOPPED, COULD CAUSE EXCESSIVE DISCHARGE
                                        # OF RESERVOIRS AND LOSS OF BRAKE RETRACTION
                                        # ABILITY
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
                    PodStatus.sensor_data['Brake_Pressure'] = 0 # DEBUG LINE
                    PodStatus.commands['Vent_Sol'] = 1     # CLOSE BRAKE VENT SOLENOID
                    print("Closing Vent Sol")
                if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] < 20:
                    PodStatus.commands['Res1_Sol'] = 1     # OPEN RES#1 SOLENOID
                    print("Opening Res#1, pausing for 2 seconds.")
                    sleep(2)
                    if PodStatus.sensor_data['Brake_Pressure'] < 177:
                        print("Unable to achieve brake retraction pressure.")
                        print("Sending Abort signal.")
                        PodStatus.Abort = True
                    else:
                        print("Brakes retracted, closing Res1 solenoid.")
                        PodStatus.commands['Res1_Sol'] = 0     # CLOSE RES#1 SOLENOID
                        transition()
                else:
                    print("Why are you here?")
                    print("Vent_Sol: " + str(PodStatus.Vent_Sol))
                    print("Brake Pressure: " + str(PodStatus.sensor_data['Brake_Pressure']))
                    x = input("1 to Abort, 2 to continue")
                    if x == '1':
                        PodStatus.Abort = True


    # CRAWLING
    # ONLY way to transition() is if LIDAR < 90ft.  Probably needs a 2nd/3rd stop point (time/dist)
    elif PodStatus.state == 6:
        PodStatus.spacex_state = 6

        # ACCEL UP TO MAX G within 2%
        if PodStatus.accel < (0.98 * PodStatus.para_max_accel)\
                and PodStatus.speed < PodStatus.para_max_crawl_speed:
            PodStatus.throttle = PodStatus.throttle * 1.01
        elif PodStatus.accel > (1.02*PodStatus.para_max_accel)\
                or PodStatus.speed > PodStatus.para_max_crawl_speed:
            PodStatus.throttle = PodStatus.throttle * 0.99

        if PodStatus.sensor_data['LIDAR'] < 90:
            print("LIDAR is less than 90 feet")
            transition()

    # BRAKE, FINAL
    elif PodStatus.state == 7:
        PodStatus.spacex_state = 5
        print("Entering final braking state.")

        PodStatus.throttle = 0

        #     PodStatus.stopped_time = 0
        if PodStatus.speed > 0.5:
            if PodStatus.commands['Vent_Sol'] == 1:    # OPEN BRAKE VENT SOLENOID
                print("Opening Vent Sol")
                PodStatus.commands['Vent_Sol'] = 0

        # TRANSITION TO S2A
        else:
            transition()

    else:
        print("Invalid pod state found: " + str(PodStatus.state))
        print("Quitting")
        PodStatus.Fault = True
        x = input("1 to Quit, 2 to reset to S2A, 3 to reset to BrakingLow(7) ")
        if x == '1': PodStatus.Quit = True
        elif x == '2': PodStatus.state = PodStatus.SafeToApproach
        else: PodStatus.state = PodStatus.BrakingLow

def transition():
    """
    This functions defines the ONLY way to perform a state transition (not including aborts).
    """
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
        PodStatus.state = 7
        PodStatus.Fault = True
        PodStatus.Quit = True

def abort():
    """
    Determines how the pod will abort from each specific state.  Generally, the pod will be
    sent to the final braking state (7), skipping the state (6) crawling reconfigurations and
    sending the pod back to S2A as soon as it comes to a stop.
    """
    if PodStatus.state == 1:          # S2A STATE FUNCTIONS
        print("Abort attempted in S2A.")
        print("No action performed.")

    elif PodStatus.state == 3:          # LAUNCH STATE FUNCTIONS
        print("ABORTING from 3 to 7")
        PodStatus.state = 7

    elif PodStatus.state == 5:
        print("ABORTING from 5 to 7")
        PodStatus.state = 7

    elif PodStatus.state == 6:
        print("ABORTING FROM 6 to 7")
        PodStatus.state = 7

    elif PodStatus.state == 7:
        if PodStatus.speed > 0.1:
            print("Waiting for pod to stop.")
        else:
            PodStatus.state = 1
    else:
        PodStatus.state = 1

def write_file():
    """
    Stores sensor_data and commands dicts to onboard SD card at the specified rate (10Hz)
    """

    if (clock() - PodStatus.log_lastwrite) < (1/PodStatus.log_rate):
        pass

    else:
        file = open(os.path.join('logs/', PodStatus.file_name), 'a')
        with file:
            ### Log sensor_data
            for key in PodStatus.sensor_data:
                if str(key) in PodStatus.abort_ranges[PodStatus.state]:
                    fault_code = PodStatus.abort_ranges[PodStatus.state][str(key)]['Fault']
                else:
                    fault_code = 0

                line = str(key) + '\t' + str(PodStatus.sensor_data[str(key)]) + '\t' \
                       + str(int(fault_code)) + '\t' + str(round(clock(),2)) + '\n'
                file.write(line)
            ### Log commands
            for key in PodStatus.commands:
                line = str(key) + '\t' + str(PodStatus.commands[str(key)]) + '\t\t' + str(round(clock(),2)) + '\n'
                file.write(line)
            ### Log pod state variables
            line = 'state' + '\t' + str(PodStatus.state) + '\t\t' + str(round(clock(),2)) + '\n' \
                    + 'spacex_state' + '\t' + str(PodStatus.spacex_state) + '\t\t' + str(round(clock(),2)) + '\n' \
                    + 'total_faults' + '\t' + str(PodStatus.total_faults) + '\t\t' + str(round(clock(),2)) + '\n' \
                    + 'throttle' + '\t' + str(PodStatus.throttle) + '\t\t' + str(round(clock(), 2)) + '\n' \
                    + 'distance' + '\t' + str(PodStatus.distance) + '\t\t' + str(round(clock(), 2)) + '\n' \
                    + 'speed' + '\t' + str(PodStatus.speed) + '\t\t' + str(round(clock(), 2)) + '\n' \
                    + 'accel' + '\t' + str(PodStatus.accel) + '\t\t' + str(round(clock(), 2)) + '\n'
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

    # DEBUG...REMOVE BEFORE FLIGHT
    print("Quitting")

