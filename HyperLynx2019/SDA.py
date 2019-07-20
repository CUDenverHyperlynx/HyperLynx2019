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

'''
Instead of print statements as error messages lets create function calls for each error
Make a function called error_log() and it will be passed a string that we can customize
Then we receive a time stamped log file as an output
Each main loop will have a separate timestamp
'''

from time import clock
import socket, struct
import numpy
import datetime
import os, psutil
import pickle
#from argparse import ArgumentParser
#import smbus
import Hyperlynx_ECS, flight_sim
from network_transfer.libclient import BaseClient
from Client import send_server
import timeouts
import can_bms


class Status():
    # Definition of State Numbers
    SafeToApproach = 1
    PreLaunch = 2
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7

    # Init dictionaries
    abort_ranges = {}           # Dict of dicts (below)
    abort_ranges[SafeToApproach] = {}
    abort_ranges[PreLaunch] = {}
    abort_ranges[Launching] = {}
    abort_ranges[BrakingHigh] = {}
    abort_ranges[Crawling] = {}
    abort_ranges[BrakingLow] = {}
    commands = {}               # Contains all possible inputs from GUI
    cmd_int = {}
    cmd_ext = {}
    sensor_data = {}            # Contains all inputs from I2C/CAN buses
    sensor_filter = {}
    true_data = {}

    poll_oldtime = 0    # Vars for poll_sensors for integrating distance from speed
    poll_newtime = 0
    poll_interval = 0

    def __init__(self):        # BOOT INIT
        self.init = False
        self.flight_sim = False
        self.wheel_diameter = 14.2 / 12 # [ft] define drive wheel diameter
        self.wheel_circum = numpy.pi * self.wheel_diameter
        self.StartTime = clock()
        self.HV = False                     # Current state of HV system (True(1) or False(0))
        self.Brakes = 1                 # Current state of brakes (1 = <177psi, 0 = >177psi)
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
        self.para_max_tube_length = 0   # [ft] Max track length
        self.para_max_crawl_speed = -1  # [ft/s] Maximum crawling speed.  Init to -1 to allow
                                        # crew to set 0 speed for crawling state

        self.IMU_bad_time = None
        self.V_bad_time = None

        self.state_timeout = [0,0,0,0,0,0,0,0]
        self.state_timeout_limits = timeouts.get()
        self.filter_length = 10         # Moving average for sensor data filter

        # SPACEX CONFIG DATA
        self.spacex_state = 0
        self.spacex_team_id = 69
        self.spacex_server_ip = '192.168.0.1'
        self.spacex_server_port = 3000
        self.spacex_rate = 40               # [Hz] rate of spacex data burst
        self.spacex_lastsend = 0

        # I2C init
        self.IMU_init_range = 0.05
        self.sensor_poll = Hyperlynx_ECS.HyperlynxECS()
        self.sensor_poll.initializeSensors()
        self.sensor_poll.initializeIO()

        # DEBUG init for script:
        self.Quit = False

        # Set filter on priority data:
        self.filter_items = ['IMU1_X', 'IMU1_Y', 'IMU1_Z', 'IMU2_X', 'IMU2_Y',
                             'IMU2_Z', 'LIDAR', 'Brake_Pressure']

        # init True values for Distance, Velocity, and Acceleration, with moving average queue, true value, and dev
        self.true_data = {'D': {'q': [], 'val': 0, 'std_dev': 0},
                          'V': {'q': [], 'val': 0, 'std_dev': 0},
                          'A': {'q': [], 'val': 0, 'std_dev': 0},
                          'stripe_count':0}

        # Pod Abort conditions init:
        self.Fault = False
        self.Abort = False

        # INITIATE STATE TO S2A
        self.state = self.SafeToApproach

        # INITIATE LOG RATE INFO
        self.log_lastwrite = clock()            # Saves last time of file write to control log rate
        self.log_rate = 10                      # Hz

    def create_log(self):
        ### Create log file ###
        date = datetime.datetime.today()
        new_number = str(date.year) + str(date.month) + str(date.day) \
                     + str(date.hour) + str(date.minute) + str(date.second)
        self.file_name = 'log_' + new_number
        file = open(os.path.join('logs/', self.file_name), 'a')
        columns = ['Label', 'Value', 'Fault', 'Time']
        with file:
            file.write('\t'.join(map(lambda column_title: "\"" + column_title + "\"", columns)))
            file.write("\n")
        file.close()
        print("Log file created: " + str(self.file_name))
    
    def data_dump(self):
        data_dict = {}
        data_dict['pos'] = self.true_data['D']['val']
        data_dict['stp_cnt'] = self.true_data['stripe_count']
        data_dict['spd'] = self.true_data['V']['val']
        data_dict['accl'] = self.true_data['A']['val']
        data_dict['IMU1_Z'] = self.sensor_filter['IMU1_Z']['val']
        data_dict['IMU2_z'] = self.sensor_filter['IMU2_Z']['val']
        data_dict['thrtl'] = self.throttle
        data_dict['lidar'] = self.sensor_filter['LIDAR']['val']
        return data_dict


def init():
    # Create Abort Range and init sensor_data Dictionary from template file
    abort_names = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                   dtype=str)
    abort_vals = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 12))

    # Assign abort conditions to each state
    for i in range(0, len(abort_names)):
        if not str(abort_names[i]) in PodStatus.sensor_data:
            PodStatus.sensor_data[abort_names[i]] = 0
        if not str(abort_names[i]) in PodStatus.sensor_filter:
            PodStatus.sensor_filter[abort_names[i]] = {'q': [], 'val': 0, 'mean': 0, 'true': 0}

        if abort_vals[i, 2] == 1:
            PodStatus.abort_ranges[PodStatus.SafeToApproach][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                      'High': abort_vals[i, 1],
                                                                      'Trigger': abort_vals[i, 9],
                                                                      'Fault': abort_vals[i, 10]
                                                                      }
        if abort_vals[i, 4] == 1:
            PodStatus.abort_ranges[PodStatus.Launching][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                 'High': abort_vals[i, 1],
                                                                 'Trigger': abort_vals[i, 9],
                                                                 'Fault': abort_vals[i, 10]
                                                                 }
            PodStatus.abort_ranges[PodStatus.PreLaunch] = PodStatus.abort_ranges[PodStatus.Launching]
        if abort_vals[i, 5] == 1:
            PodStatus.abort_ranges[PodStatus.BrakingHigh][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                   'High': abort_vals[i, 1],
                                                                   'Trigger': abort_vals[i, 9],
                                                                   'Fault': abort_vals[i, 10]
                                                                   }
        if abort_vals[i, 7] == 1:
            PodStatus.abort_ranges[PodStatus.Crawling][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                'High': abort_vals[i, 1],
                                                                'Trigger': abort_vals[i, 9],
                                                                'Fault': abort_vals[i, 10]
                                                                }
        if abort_vals[i, 8] == 1:
            PodStatus.abort_ranges[PodStatus.BrakingLow][abort_names[i]] = {'Low': abort_vals[i, 0],
                                                                  'High': abort_vals[i, 1],
                                                                  'Trigger': abort_vals[i, 9],
                                                                  'Fault': abort_vals[i, 10]
                                                                  }

    PodStatus.cmd_int = {"Abort": 0,
                         "HV": 0,
                         'Launch': 0,
                         "Vent_Sol": 0,
                         "Res1_Sol": 0,
                         "Res2_Sol": 0,
                         "MC_Pump": 0}

    PodStatus.cmd_ext = {"Abort": 0,
                         "HV": 0,
                         'Launch': 0,
                         "Vent_Sol": 0,
                         "Res1_Sol": 0,
                         "Res2_Sol": 0,
                         "MC_Pump": 0}

    ## CHECK IMU INIT
    print("Checking IMUs")
    poll_sensors()
    filter_data()
    if abs(PodStatus.sensor_filter['IMU1_Z']['val']) < PodStatus.IMU_init_range and \
            abs(PodStatus.sensor_filter['IMU2_Z']['val']) < PodStatus.IMU_init_range:
        print("Both IMUs valid.")
        PodStatus.init = True
    else:
        print('IMU1_Z: ' + str(PodStatus.sensor_data['IMU1_Z']))
        print('IMU2_Z: ' + str(PodStatus.sensor_data['IMU2_Z']))
        print("IMU init failed.")
        PodStatus.Fault = True

    PodStatus.create_log()

    ## Confirm boot info ##
    print("Pod init complete, State: " + str(PodStatus.state))


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
    # None Yet Added


    ### I2C DATA ###

    # If you want to run the flight sim:
    if PodStatus.flight_sim is True:

        # PodStatus.sensor_data['Brake_Pressure'] = PodStatus.sensor_poll.getBrakePressure()
        PodStatus.sensor_data['LVBatt_Temp'] = PodStatus.sensor_poll.getBatteryTemp()
        PodStatus.sensor_data['LVBatt_Current'] = 4
        PodStatus.sensor_data['LVBatt_Voltage'] = 12
        PodStatus.sensor_data['PV_Left_Temp'] = PodStatus.sensor_poll.getBMEtemperature(2)
        PodStatus.sensor_data['PV_Left_Pressure'] = PodStatus.sensor_poll.getBMEpressure(2)
        PodStatus.sensor_data['PV_Right_Temp'] = PodStatus.sensor_poll.getBMEtemperature(1)
        PodStatus.sensor_data['PV_Right_Pressure'] = PodStatus.sensor_poll.getBMEpressure(1)
        PodStatus.sensor_data['Ambient_Pressure'] = PodStatus.sensor_poll.getTubePressure()
        tempAccel1 = PodStatus.sensor_poll.getAcceleration(1)
        PodStatus.sensor_data['IMU1_X'] = tempAccel1[1]
        PodStatus.sensor_data['IMU1_Y'] = tempAccel1[2]
        # PodStatus.sensor_data['IMU1_Z'] = tempAccel1[0]
        tempAccel2 = PodStatus.sensor_poll.getAcceleration(2)
        PodStatus.sensor_data['IMU2_X'] = tempAccel2[1]
        PodStatus.sensor_data['IMU2_Y'] = tempAccel2[2]
        # PodStatus.sensor_data['IMU2_Z'] = tempAccel2[0]
        PodStatus.sensor_data['LIDAR'] = PodStatus.para_max_tube_length - PodStatus.true_data['D']['val']
        if PodStatus.sensor_data['LIDAR'] > 150: PodStatus.sensor_data['LIDAR'] = 150

        flight_sim.sim(PodStatus)

    else:
        # Uncomment Brake Pressure for pulling in actual data when we have this set up
        # PodStatus.sensor_data['Brake_Pressure'] = PodStatus.sensor_poll.getBrakePressure()
        PodStatus.sensor_data['LVBatt_Temp'] = PodStatus.sensor_poll.getBatteryTemp()
        PodStatus.sensor_data['LVBatt_Current'] = PodStatus.sensor_poll.getCurrentLevel()
        PodStatus.sensor_data['LVBatt_Voltage'] = PodStatus.sensor_poll.getVoltageLevel()
        PodStatus.sensor_data['PV_Left_Temp'] = PodStatus.sensor_poll.getBMEtemperature(2)
        PodStatus.sensor_data['PV_Left_Pressure'] = PodStatus.sensor_poll.getBMEpressure(2)
        PodStatus.sensor_data['PV_Right_Temp'] = PodStatus.sensor_poll.getBMEtemperature(1)
        PodStatus.sensor_data['PV_Right_Pressure'] = PodStatus.sensor_poll.getBMEpressure(1)
        PodStatus.sensor_data['Ambient_Pressure'] = PodStatus.sensor_poll.getTubePressure()
        tempAccel1 = PodStatus.sensor_poll.getAcceleration(1)
        PodStatus.sensor_data['IMU1_X'] = tempAccel1[1]
        PodStatus.sensor_data['IMU1_Y'] = tempAccel1[2]
        PodStatus.sensor_data['IMU1_Z'] = tempAccel1[0]
        tempAccel2 = PodStatus.sensor_poll.getAcceleration(2)
        PodStatus.sensor_data['IMU2_X'] = tempAccel2[1]
        PodStatus.sensor_data['IMU2_Y'] = tempAccel2[2]
        PodStatus.sensor_data['IMU2_Z'] = tempAccel2[0]
        PodStatus.sensor_data['LIDAR'] = PodStatus.sensor_poll.getLidarDistance()

    if abs(PodStatus.sensor_data['IMU1_Z']) > 20:
        PodStatus.sensor_data['IMU1_Z'] = 0
    if abs(PodStatus.sensor_data['IMU2_Z']) > 20:
        PodStatus.sensor_data['IMU2_Z'] = 0

    ### RPI DATA ###
    rpi_data = psutil.disk_usage('/')
    PodStatus.sensor_data['RPi_Disk_Space_Free'] = rpi_data.free / (1024 ** 2)
    PodStatus.sensor_data['RPi_Disk_Space_Used'] = rpi_data.used / (1024 ** 2)
    PodStatus.sensor_data['RPi_Proc_Load'] = round(psutil.cpu_percent(),1)
    rpi_data2 = psutil.virtual_memory()
    PodStatus.sensor_data['RPi_Mem_Load'] = rpi_data2.percent
    PodStatus.sensor_data['RPi_Mem_Free'] = rpi_data2.free / 2 ** 20
    PodStatus.sensor_data['RPi_Mem_Used'] = rpi_data2.used / 2 ** 20
    # temp = os.popen("vcgencmd measure_temp").readline()
    # temp = temp.replace("temp=",'')
    # temp = temp.replace("'C",'')
    # PodStatus.sensor_data['RPi_Temp'] = temp

    ### SPACEX DATA ###

    ### CONVERT DATA ###
    PodStatus.sensor_poll.statusCheck()

    if PodStatus.sensor_data['Brake_Pressure'] > 177:
        PodStatus.Brakes = False
    else:
        PodStatus.Brakes = True
    PodStatus.Vent_Sol = PodStatus.cmd_int['Vent_Sol']
    PodStatus.Res1_Sol = PodStatus.cmd_int['Res1_Sol']
    PodStatus.Res2_Sol = PodStatus.cmd_int['Res2_Sol']

    # Integrate distance
    if PodStatus.state != 1:
        PodStatus.true_data['D']['val'] += PodStatus.true_data['V']['val'] * PodStatus.poll_interval

    # Update MET
    if PodStatus.MET > 0:
        PodStatus.MET = clock()-PodStatus.MET_starttime


def filter_data():
    """ Filters sensor data based on moving average.
    """
    ### Search all sensor_data points
    for key in PodStatus.sensor_data:

        ### Refine search to just priority items specified in PodStatus.init()
        if key in PodStatus.filter_items:

            # If queue is not full, fill queue
            if len(PodStatus.sensor_filter[str(key)]['q']) < PodStatus.filter_length:
                PodStatus.sensor_filter[str(key)]['q'] = numpy.append(PodStatus.sensor_filter[str(key)]['q'],
                                                                      PodStatus.sensor_data[str(key)])
            else:
                PodStatus.sensor_filter[str(key)]['std_dev'] = 3*numpy.std(PodStatus.sensor_filter[str(key)]['q'])
                PodStatus.sensor_filter[str(key)]['mean'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])

                # if new value is inside range of std_dev (hence valid), then add to q
                if abs(PodStatus.sensor_data[str(key)]-PodStatus.sensor_filter[str(key)]['q'][(PodStatus.filter_length-1)]) <= \
                        PodStatus.sensor_filter[str(key)]['std_dev']:

                    # shift q values over
                    for i in range(0, (len(PodStatus.sensor_filter[str(key)]['q'])-1)):
                        PodStatus.sensor_filter[str(key)]['q'][i] = PodStatus.sensor_filter[str(key)]['q'][i+1]
                    # add new value to end of queue
                    PodStatus.sensor_filter[str(key)]['q'][(PodStatus.filter_length-1)] = PodStatus.sensor_data[str(key)]
                    # set the filtered value to the mean of the new queue
                else:
                    print('Did not add ' + str(key) + ' to q: ' + str(PodStatus.sensor_data[str(key)]) + str(PodStatus.MET))
                    print('Current std dev: ' + str(PodStatus.sensor_filter[str(key)]['std_dev']))
            PodStatus.sensor_filter[str(key)]['val'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])


def sensor_fusion():
    """ Combines various filtered sensor data to a common solution."""

    ### BEGIN ACCELERATION FUSION
    # If queue is not full, fill queue

    PodStatus.true_data['A']['val'] = numpy.mean([PodStatus.sensor_filter['IMU1_Z']['val'],
                                                  PodStatus.sensor_filter['IMU2_Z']['val']])

    # good_IMUs = []  # reset good_IMUs to empty set
    # if len(PodStatus.true_data['A']['q']) < PodStatus.filter_length:
    #     # Add mean of IMU values to
    #     print('Adding to Q')
    #     PodStatus.true_data['A']['q'] = numpy.append(PodStatus.true_data['A']['q'],
    #                                     numpy.mean([PodStatus.sensor_filter['IMU1_Z']['val'],
    #                                                 PodStatus.sensor_filter['IMU2_Z']['val']]))
    #
    # else:
    #     PodStatus.true_data['A']['std_dev'] = numpy.std(PodStatus.true_data['A']['q'])
    #     numpy.append(good_IMUs, PodStatus.sensor_filter['IMU1_Z']['val'])
    #     numpy.append(good_IMUs, PodStatus.sensor_filter['IMU2_Z']['val'])
    #
    #     # # determine valid IMU data
    #     # # if new sensor_filter data is within 2*std_dev of true q, include in good_IMUs calc
    #     # if abs(PodStatus.sensor_filter['IMU1_Z']['val']-numpy.mean(PodStatus.true_data['A']['q'])) < \
    #     #         3 * PodStatus.true_data['A']['std_dev']:
    #     #     numpy.append(good_IMUs, PodStatus.sensor_filter['IMU1_Z']['val'])
    #     # else:
    #     #     print('Fusion: IMUZ_1 Data Bad\n')
    #     #     print('IMUZ Value ' + str(abs(PodStatus.sensor_filter['IMU1_Z']['val'])) + '\n')
    #     #     print('A Mean ' + str(numpy.mean(PodStatus.true_data['A']['q'])) + '\n')
    #     #     print('A two times STD ' + str(3 * PodStatus.true_data['A']['std_dev']) + '\n')
    #     # if abs(PodStatus.sensor_filter['IMU2_Z']['val'] - numpy.mean(PodStatus.true_data['A']['q'])) < \
    #     #         3 * PodStatus.true_data['A']['std_dev']:
    #     #     numpy.append(good_IMUs, PodStatus.sensor_filter['IMU2_Z']['val'])
    #     # else:
    #     #     print('Fusion: IMUZ_2 Data Bad\n')
    #     #     print('IMUZ Value ' + str(abs(PodStatus.sensor_filter['IMU2_Z']['val'])) + '\n')
    #     #     print('A Mean ' + str(numpy.mean(PodStatus.true_data['A']['q'])) + '\n')
    #     #     print('A two times STD ' + str(3 * PodStatus.true_data['A']['std_dev']) + '\n')
    #
    #     # if good_IMUs is not empty set, take mean value and add to true_data q
    #     if good_IMUs:
    #         print("Good data found for A")
    #         # reset IMU_bad_time timer
    #         PodStatus.IMU_bad_time_elapsed = 0
    #         PodStatus.IMU_bad_time = None
    #
    #         # Set true value to mean of good_IMUs array
    #         PodStatus.true_data['A']['val'] = numpy.mean(good_IMUs)
    #
    #         # add valid data to true_data q
    #         # shift q values over
    #         for i in range(0, (len(PodStatus.true_data['A']['q']) - 1)):
    #             PodStatus.true_data['A']['q'][i] = PodStatus.true_data['A']['q'][i + 1]
    #         # add new value to end of queue
    #         PodStatus.true_data['A']['q'][(PodStatus.filter_length - 1)] = PodStatus.true_data['A']['val']
    #
    #     # if no good IMU data, start or progress the bad IMU data timer
    #     else:
    #         if not PodStatus.IMU_bad_time:
    #             PodStatus.IMU_bad_time = clock()
    #             print("Bad IMU data, starting 2 second clock at " + str(PodStatus.IMU_bad_time))
    #
    #         else:
    #             PodStatus.IMU_bad_time_elapsed = clock()-PodStatus.IMU_bad_time
    #             print("Bad IMU data, elapsed time: " + str(PodStatus.IMU_bad_time_elapsed))


    ### END ACCELERATION FUSION

    ### BEGIN VELOCITY FUSION
    PodStatus.true_data['V']['val'] = PodStatus.sensor_data['SD_MotorData_MotorRPM'] * PodStatus.wheel_circum / 60

    if PodStatus.true_data['V']['val'] < 0: PodStatus.true_data['V']['val'] = 0
    # If queue is not full, fill queue
    # if len(PodStatus.true_data['V']['q']) < PodStatus.filter_length:
    #     # Add mean of IMU values to
    #     PodStatus.true_data['V']['q'] = numpy.append(PodStatus.true_data['V']['q'],
    #                                                  (PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val'] * \
    #                                                   PodStatus.wheel_circum / 60))
    #
    # else:
    #     # Estimate new velocity
    #     Vdr = PodStatus.true_data['V']['val'] + PodStatus.poll_interval * PodStatus.true_data['A']['val'] * 32.174
    #
    #     # Set std_dev
    #     PodStatus.true_data['V']['std_dev'] = numpy.std(PodStatus.true_data['V']['q'])
    #
    #     # Evaluate new data compared to true q and std dev
    #     # if abs(PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val'] - numpy.mean(PodStatus.true_data['V']['q'])) < \
    #     #         2 * PodStatus.true_data['V']['std_dev']:
    #     #     PodStatus.V_bad_time_elapsed = 0
    #     #     PodStatus.V_bad_time = None
    #
    #     PodStatus.true_data['V']['val'] = numpy.mean([Vdr, \
    #                                                   (PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val'] * PodStatus.wheel_circum / 60)])

        # if new data is invalid:
        # else:
        #     # RUN TRACTION CONTROL FUNCTION
        #
        #     # Start or progress bad V data timer
        #     if not PodStatus.V_bad_time:
        #         PodStatus.V_bad_time = clock()
        #         print("Bad V data, starting clock at " + str(PodStatus.IMU_bad_time))
        #
        #     else:
        #         PodStatus.V_bad_time_elapsed = clock()-PodStatus.V_bad_time
        #         print("Bad V data, elapsed time: " + str(PodStatus.V_bad_time_elapsed))
    ### END VELOCITY FUSION

    ### BEGIN DISTANCE FUSION
    PodStatus.true_data['D']['val'] = PodStatus.true_data['D']['val'] + PodStatus.poll_interval * \
        PodStatus.true_data['V']['val']

    PodStatus.true_data['stripe_count'] = numpy.maximum(PodStatus.sensor_data['LST_Left'],
                                                    PodStatus.sensor_data['LST_Right'])

    PodStatus.D_diff = PodStatus.true_data['D']['val'] - PodStatus.stripe_count

    ### Unnecessary block, since we count stripes independently and take max as true_data
    # if PodStatus.true_data['D']['val'] > 25:
    #     temp_count = PodStatus.true_data['D']['val'] / 100
    #     temp_dec = abs(temp_count - numpy.around(temp_count))
    #     if temp_dec < 0.2 and PodStatus.D_diff > 20:
    #         print("Looking for stripe")
    #         if PodStatus.sensor_data['LST_Left'] == temp_count:
    #             print("Left stripe counted!")
    #             PodStatus.stripe_count = PodStatus.sensor_data['LST_Left']
    #             PodStatus.sensor_data['LST_Right'] = PodStatus.sensor_data['LST_Left']
    #         elif PodStatus.sensor_data['LST_Right'] == temp_count:
    #             print("Right stripe counted!")
    #             PodStatus.stripe_count = PodStatus.sensor_data['LST_Right']
    #             PodStatus.sensor_data['LST_Left'] = PodStatus.sensor_data['LST_Right']
    #         else:
    #             print("No stripe counted, still waiting.")
    #
    #     else:
    #         pass


    ### END DISTANCE FUSION

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
        # if data is not being filtered, record abort criteria based on raw
        if not key in PodStatus.sensor_filter[str(key)]:
            # if out of range, log 'Fault' key as 1
            if PodStatus.sensor_data[str(key)] < PodStatus.abort_ranges[PodStatus.state][str(key)]['Low'] \
                    or PodStatus.sensor_data[str(key)] > PodStatus.abort_ranges[PodStatus.state][str(key)]['High']:
                ### DEBUG PRINT
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_data[str(key)]))
                print("Range:\t" + str(PodStatus.abort_ranges[PodStatus.state][str(key)]['Low']) +
                      " to " + str(PodStatus.abort_ranges[PodStatus.state][str(key)]['High']))
                PodStatus.abort_ranges[PodStatus.state][str(key)]['Fault'] = 1
        # if data is being filtered, record abort criteria based on filtered data
        else:
            # if out of range, log 'Fault' key as 1
            if PodStatus.sensor_filter[str(key)]['val'] < PodStatus.abort_ranges[PodStatus.state][str(key)]['Low'] \
                    or PodStatus.sensor_filter[str(key)]['val'] > PodStatus.abort_ranges[PodStatus.state][str(key)]['High']:
                ### DEBUG PRINT
                print("Pod Fault!\tSensor: " + str(key))
                print("Value:\t" + str(PodStatus.sensor_filter[str(key)]['val']))
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
        print("Number of Faults: \t" + str(int(PodStatus.total_faults)))
    else:
        PodStatus.Fault = False

    if PodStatus.total_triggers > 0:
        ### DEBUG PRINT TRIGGERS
        print("ABORT TRIGGERS FOUND: \t" + str(int(PodStatus.total_triggers)))
        print("FLAGGING ABORT == TRUE")
        PodStatus.Abort = True         # This is the ONLY location an abort can be reached during this function
        # PodStatus.cmd_int['Abort'] = 1


def rec_data():

    ### CAN BUS RECEIVE ###
    bms_data = can_bms.run()

    ###__ACTUAL GUI__###
    if gui == '2':
        ### RECEIVE DATA FROM GUI ###
        # Need to receive: cmd_ext{} and para{} into temp values.
        # If state = 1, then load all cmd_ext{} and para_ into the PodStatus dicts.
        # If state != 1, then *only* load the cmd_ext['Abort'] value to the PodStatus.cmd_int['Abort'] var.
        # Have a running clock for GUI; if loss of connection > 2 seconds, will abort

        pass

    ###DEBUG CONSOLE GUI###
    if gui == '1':
        """
        During DEBUG, this function is a mock GUI for testing SDA.
        """
        PodStatus.para_BBP = 3300
        PodStatus.para_max_speed = 300
        PodStatus.para_max_accel = 0.7
        PodStatus.para_max_time = 15
        PodStatus.para_max_crawl_speed = 20
        PodStatus.para_max_tube_length = 4150

        ### TEST SCRIPT FOR FAKE COMMAND DATA / GUI
        print("\n******* POD STATUS ******\n"
            "\tState:               " + str(PodStatus.state) + "\t\t")
        print("\tFlight Sim:        " + str(PodStatus.flight_sim) + "\t\t")
        print("\tPod Clock Time:    " + str(round(PodStatus.MET,3)) + "\t")
        print("\tFault:             " + str(PodStatus.Fault) + "\t\t")
        print("\tAbort Flag         " + str(PodStatus.Abort) + "\t\t")
        print("\t2. HV System       " + str(PodStatus.HV) + "\t\t")
        print("\tBrakes:            " + str(PodStatus.Brakes) + "\t\n"
            "\t3. Vent Solenoid:    " + str(PodStatus.Vent_Sol) + "\t\n"
            "\t4. Res1 Solenoid:    " + str(PodStatus.Res1_Sol) + "\t\n"
            "\t5. Res2 Solenoid:    " + str(PodStatus.Res2_Sol) + "\t\n"
            "\t6. MC Pump:          " + str(PodStatus.MC_Pump) + "\t\n"
            "\t7. Flight BBP:       " + str(PodStatus.para_BBP) + "\t\n"
            "\t8. Flight Speed:     " + str(PodStatus.para_max_speed) + "\t\n"
            "\t9. Flight Accel:     " + str(PodStatus.para_max_accel) + "\t\n"
            "\t10.Flight Time:      " + str(PodStatus.para_max_time) + "\t\n"
            "\t11.Flight Crawl Speed\t" + str(PodStatus.para_max_crawl_speed) + "\t\n"
            "\tThrottle:\t" + str(round(PodStatus.throttle,2)) + "\t\n"
            "\t\tVelocity:\t" + str(round(PodStatus.true_data['V']['val'],2)) + "\t\n"
            "\t\tDistance:\t" + str(round(PodStatus.true_data['D']['val'],2)) + "\t\n"
            "*************************")

        if PodStatus.state == PodStatus.SafeToApproach:
            print("\n*** MENU ***\n"
                  "\t(L) Launch\n"
                  "\t(R) Reset Abort Flag\n"
                  "\t(Q) Quit\n"
                  "\t(FS) Activate Flight Sim\n"
                  "\t\tType line number or letter command to change values.\n")
            print(str('Brake Pressure: ' + str(PodStatus.sensor_data['Brake_Pressure'])))
            a = input('Enter choice: ')
            if a == '2':
                if PodStatus.HV is False:
                    PodStatus.cmd_ext['HV'] = 1
                    # PodStatus.HV = True
                else:
                    PodStatus.cmd_ext['HV'] = 0
                    # PodStatus.HV = False
            elif a == '3':
                if PodStatus.cmd_ext['Vent_Sol'] == 0:
                    PodStatus.cmd_ext['Vent_Sol'] = 1           # Brake Vent opens
                    # PodStatus.Vent_Sol = 0
                    # PodStatus.sensor_data['Brake_Pressure'] = 15      # Change brake pressure to atmo
                else:
                    PodStatus.cmd_ext['Vent_Sol'] = 0
                    # PodStatus.Vent_Sol = 1
            elif a == '4':
                if PodStatus.cmd_ext['Res1_Sol'] == 0:
                    PodStatus.cmd_ext['Res1_Sol'] = 1

                else:
                    PodStatus.cmd_ext['Res1_Sol'] = 0
            elif a == '5':
                if PodStatus.cmd_ext['Res2_Sol'] == 1:
                    PodStatus.cmd_ext['Res2_Sol'] = 0
                else:
                    PodStatus.cmd_ext['Res2_Sol'] = 1
            elif a == '6':
                if PodStatus.MC_Pump == 0:
                    PodStatus.cmd_ext['MC_Pump'] = 1
                else:
                    PodStatus.cmd_ext['MC_Pump'] = 0
            elif a == '7':
                PodStatus.para_BBP = float(input("Enter BBP Distance in feet: "))
            elif a == '8':
                PodStatus.para_max_speed = float(input("Enter max speed in ft/s: "))
            elif a == '9':
                PodStatus.para_max_accel = float(input("Enter max accel in g: "))
            elif a == '10':
                PodStatus.para_max_time = float(input("Enter max time in s: "))
            elif a == '11':
                PodStatus.para_max_crawl_speed = float(input("Enter max crawl speed in ft/s: "))
            elif a == 'L':
                PodStatus.cmd_ext['Launch'] = 1
            elif a == 'R':
                PodStatus.cmd_ext['Abort'] = 0
                PodStatus.stopped_time = -1
                PodStatus.MET_startime = -1
            elif a == 'Q':
                PodStatus.Quit = True
            elif a == 'FS':
                PodStatus.sensor_data['Brake_Pressure'] = 200
                PodStatus.flight_sim = True
            else:
                pass

        # Commented out for Flight Simulation purposes
        elif PodStatus.state == PodStatus.Launching:
            pass
            #print("\n*** MENU ***\n\t1. Abort\n\t2. Brake\n\t3. Quit")
            #a = input('Enter choice: ')
            #if a == '1':
            #    PodStatus.commands['Abort'] = 1
            #    abort()
            #elif a == '2':
            #     PodStatus.distance = PodStatus.para_BBP + 1
            #     print(PodStatus.distance)
            # elif a == '3':
            #     PodStatus.Quit = True
            # else:
            #     pass
        elif PodStatus.state == PodStatus.BrakingHigh:
            pass

        elif PodStatus.state == PodStatus.Crawling:
            print("\n*** MENU ***\n\t1. Abort\n\t2. Quit")
            a = input('Enter choice: ')
            if a == '1':
                PodStatus.cmd_ext['Abort'] = 1
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
    This function ensures all current cmd_int{} are run each loop.  The GUI will show it's current commands,
    as well as the pod's current commands.

    This is the ONLY function that will allow the pod to transition from S2A to Launching.
    """

    if PodStatus.state == 1:    # Load ALL commands for full GUI control
        PodStatus.cmd_int = PodStatus.cmd_ext

        ### POD WILL LAUNCH WITH THIS SECTION ###
        # Launch pod ONLY if conditions in run_state() for spacex_state are met
        if PodStatus.cmd_ext['Launch'] == 1 and PodStatus.spacex_state == 2:
            transition()
        elif PodStatus.cmd_ext['Launch'] == 1 and PodStatus.spacex_state != 2:
            print("Pod not configured for launch, resetting Launch command to 0.")
            PodStatus.cmd_ext['Launch'] = 0
            PodStatus.cmd_int['Launch'] = 0

        # Coolant Pump
        PodStatus.sensor_poll.switchCoolantPump(PodStatus.cmd_int['MC_Pump'])
        if PodStatus.cmd_ext['MC_Pump'] == 1:
            PodStatus.MC_Pump = True
        else:
            PodStatus.MC_Pump = False

    # COMMANDS FOR ALL STATES
    # Brake Solenoid operation
    PodStatus.sensor_poll.switchSolenoid(3, PodStatus.cmd_int['Vent_Sol'])
    PodStatus.Vent_Sol = bool(PodStatus.cmd_int['Vent_Sol'])
    PodStatus.sensor_poll.switchSolenoid(1, PodStatus.cmd_int['Res1_Sol'])
    PodStatus.Res1_Sol = bool(PodStatus.cmd_int['Res1_Sol'])
    PodStatus.sensor_poll.switchSolenoid(2, PodStatus.cmd_int['Res2_Sol'])
    PodStatus.Res2_Sol = bool(PodStatus.cmd_int['Res2_Sol'])

    # HV Contactors (and red LED by default)
    PodStatus.sensor_poll.switchContactor(1, PodStatus.cmd_int['HV'])
    PodStatus.sensor_poll.switchContactor(2, PodStatus.cmd_int['HV'])
    PodStatus.HV = bool(PodStatus.cmd_int['HV'])

    # Abort Command
    if PodStatus.Abort:
        abort()

    # Isolation green LED   DEBUG NEED VAR DATA FOR BMS
    # if PodStatus.sensor_data['BMS_something'] < 4.5:
    #     PodStatus.sensor_poll.switchGreenLED(0);
    # else:
    #     PodStatus.sensor_poll.switchGreenLED(1);


def spacex_data():
    """
    This function passes the required SpaceX data packet at the defined rate.
    """
    ### CONVERT DATA TO SPACEX SPECIFIED UNIT
    accel = PodStatus.true_data['A']['val'] * 3217.4        # g (unitless) to cm/s2
    speed = PodStatus.true_data['V']['val'] * 30.48         # ft/s to cm/s
    distance = PodStatus.true_data['D']['val'] * 30.48   # ft to cm

    if (clock()-PodStatus.spacex_lastsend) < (1/PodStatus.spacex_rate):
        # print("No packet sent.")
        pass
    else:
        PodStatus.spacex_lastsend = clock()
        # print("SpaceX packet sent at " + str(PodStatus.spacex_lastsend))

        ### SpaceX-provided code
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = (PodStatus.spacex_server_ip, PodStatus.spacex_server_port)
        packet = struct.pack(">BB7iI", PodStatus.spacex_team_id, PodStatus.spacex_state, int(accel),
                             int(distance), int(speed), 0, 0, 0, 0, int(PodStatus.stripe_count) // 3048)
        sock.sendto(packet, server)


def send_data():        # Sends data to TCP (GUI) and CAN (BMS/MC)

    ### Send to CAN ###

    ### Send to TCP ###
    #send_server(PodStatus)
    pass

def run_state():
    """
    This is the primary "run" function for each state.
    """

    # S2A STATE
    if PodStatus.state == 1:
        # Determine if SpaceX state = 1 (S2A) or 2 (Ready to Launch)
        if (PodStatus.Fault is False and
            PodStatus.Abort is False and
            PodStatus.HV is True and
            PodStatus.Brakes is False and
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

    # PRELAUNCH State
    elif PodStatus.state == 2:
        # Set Motor Controller Parameters
        # Emerg Brake / Active? / Forward

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True


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
        if PodStatus.true_data['A']['val'] < (0.98 * PodStatus.para_max_accel):
            if PodStatus.throttle < 1:
                PodStatus.throttle = PodStatus.throttle + 0.1
                if PodStatus.throttle > 1:
                    PodStatus.throttle = 1
        elif PodStatus.true_data['A']['val'] > (1.02*PodStatus.para_max_accel):
            if PodStatus.throttle > 0:
                PodStatus.throttle = PodStatus.throttle - 0.1
                if PodStatus.throttle < 0:
                    PodStatus.throttle = 0

        # TRANSITIONS
        if (PodStatus.true_data['D']['val'] > PodStatus.para_BBP) or \
                (PodStatus.true_data['stripe_count']*100 > PodStatus.para_BBP):
            print("Pod has crossed BBP.")
            transition()
        elif PodStatus.true_data['V']['val'] > PodStatus.para_max_speed:
            print("Pod has reached max speed.")
            transition()
        elif PodStatus.MET > PodStatus.para_max_time:
            print("Pod has exceeded max time.")
            transition()
        # TRANSITIONS FOR BAD DATA
        elif PodStatus.abort_ranges[PodStatus.state]['IMU_bad_time_elapsed']['Fault'] == 1:
            print("Transition for bad IMU data.")
            transition()

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

    # COAST (NOT USED)

    # BRAKE, HIGH SPEED
    elif PodStatus.state == 5:
        PodStatus.sensor_filter['IMU1_Z']['q'] = []
        PodStatus.sensor_filter['IMU2_Z']['q'] = []
        PodStatus.sensor_filter['Brake_Pressure']['q'] = []
        PodStatus.MET = clock()-PodStatus.MET_starttime
        PodStatus.spacex_state = 5

        PodStatus.throttle = 0  # SET THROTTLE TO 0

        if PodStatus.true_data['V']['val'] <= 0.5 and PodStatus.stopped_time <= 0:
            PodStatus.stopped_time = clock()

        # THIS VALUE NEEDS TO BE THOROUGHLY TESTED;
        # IF ERRANT SPEED VALUES > 0.5 WHILE ACTUALLY
        # STOPPED, COULD CAUSE EXCESSIVE DISCHARGE
        # OF RESERVOIRS AND LOSS OF BRAKE RETRACTION ABILITY
        if PodStatus.true_data['V']['val'] > 0.5:
            PodStatus.stopped_time = 0              # RESET STOPPED TIME
            if PodStatus.cmd_int['Vent_Sol'] == 1:    # Is brake vent closed?
                print("Opening Vent Sol")
                PodStatus.cmd_int['Vent_Sol'] = 0       # open brake vent

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

        # DO NOTHING ELSE UNTIL STOPPED


        ## RECONFIGURE FOR CRAWLING STATE

        if PodStatus.true_data['V']['val'] < 0.5 and (clock() - PodStatus.stopped_time) > 5:
            if PodStatus.cmd_int['Vent_Sol'] == 0:
                PodStatus.cmd_int['Vent_Sol'] = 1     # CLOSE BRAKE VENT SOLENOID
                print("Closing Vent Sol")
            if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] < 20:
                PodStatus.cmd_int['Res1_Sol'] = 1     # OPEN RES#1 SOLENOID
                print("Opening Res#1, pausing for 2 seconds.")

            else:
                print("Waiting for pod to achieve braking pressure")
                print("Vent_Sol: " + str(PodStatus.Vent_Sol))
                print("Res1_Sol: " + str(PodStatus.Res1_Sol))
                print("Brake Pressure: " + str(PodStatus.sensor_data['Brake_Pressure']))

            if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] > 177:
                print("Brakes retracted, closing Res1 solenoid.")
                PodStatus.cmd_int['Res1_Sol'] = 0  # CLOSE RES#1 SOLENOID
                transition()

    # CRAWLING
    # ONLY way to transition() is if LIDAR < 90ft.  Probably needs a 2nd/3rd stop point (time/dist)
    elif PodStatus.state == 6:
        PodStatus.spacex_state = 6
        PodStatus.sensor_filter['IMU1_Z']['q'] = []
        PodStatus.sensor_filter['IMU2_Z']['q'] = []
        PodStatus.sensor_filter['Brake_Pressure']['q'] = []

        # ACCEL UP TO MAX G within 2%
        if PodStatus.true_data['A']['val'] < (0.98 * PodStatus.para_max_accel)\
                and PodStatus.true_data['V']['val'] < PodStatus.para_max_crawl_speed:
            PodStatus.throttle = PodStatus.throttle + 0.05
            if PodStatus.throttle > 1:
                PodStatus.throttle = 1
        elif PodStatus.true_data['A']['val'] > (1.02*PodStatus.para_max_accel)\
                or PodStatus.true_data['V']['val'] > PodStatus.para_max_crawl_speed:
            PodStatus.throttle = PodStatus.throttle - 0.05
            if PodStatus.throttle < 0:
                PodStatus.throttle = 0

        if PodStatus.sensor_data['LIDAR'] < 90 or (PodStatus.para_max_tube_length - PodStatus.true_data['D']['val']) < 150:
            print("LIDAR is less than 90 feet")
            transition()

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

    # BRAKE, FINAL
    elif PodStatus.state == 7:
        PodStatus.spacex_state = 5
        print("Entering final braking state.")

        PodStatus.throttle = 0
        PodStatus.cmd_int['HV'] = 0

        #     PodStatus.stopped_time = 0
        if PodStatus.true_data['V']['val'] > 0.5:
            if PodStatus.cmd_int['Vent_Sol'] == 1:    # OPEN BRAKE VENT SOLENOID
                print("Opening Vent Sol")
                PodStatus.cmd_int['Vent_Sol'] = 0

        # TRANSITION TO S2A
        else:
            transition()

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

    else:
        PodStatus.state = PodStatus.BrakingLow

        #DEBUG
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
        # Reconfig1-2-3 States
        # Close Brake Vent, open res#1 solenoid, close res#1 solenoid
        # APPLIES TO CONFIGS WITH NO RES IN-LINE REGULATOR
        if PodStatus.cmd_int['Vent_Sol'] == 0:
            PodStatus.cmd_int['Vent_Sol'] = 1     # CLOSE VENT SOL
        elif PodStatus.Vent_Sol:
            PodStatus.cmd_int['Res1_Sol'] = 1       # OPEN RES#1 SOL
        elif PodStatus.Res1_Sol:
            if PodStatus.sensor_data['Brake_Pressure'] > 177:
                PodStatus.cmd_int['Res1_Sol'] = 0   # CLOSE RES#1 SOL
                PodStatus.state = 6
                print("TRANS: BRAKE(5) to CRAWLING(6)")

        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

    elif PodStatus.state == 6:
        ### RECONFIG 4 STATE
        # TELL SD100 TO CHANGE DRIVE MODE, SET EMERG BRAKE
        # Timeout
        if PodStatus.state_timeout[PodStatus.state] == 0:
            PodStatus.state_timeout[PodStatus.state] = clock()
        PodStatus.state_timeout[PodStatus.state] = clock() - PodStatus.state_timeout[PodStatus.state]

        # Timeout Transition
        if PodStatus.state_timeout[PodStatus.state] > PodStatus.state_timeout_limits[PodStatus.state]:
            PodStatus.Fault = True
            PodStatus.Abort = True

        PodStatus.state = 7
        print("TRANS: CRAWLING(6) TO BRAKE(7)")

    elif PodStatus.state == 7:
        PodStatus.state = 1
        print("TRANS: BRAKE(7) TO S2A(1)")
        print("Creating new log file.")
        PodStatus.create_log()

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
        print("Abort flagged in S2A.")

    elif PodStatus.state == 2:          # PreLaunch Abort
        PodStatus.state = 7

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
            for key in PodStatus.cmd_ext:
                line = 'ext_' + str(key) + '\t' + str(PodStatus.cmd_ext[str(key)]) + '\t\t' + str(round(clock(),2)) + '\n'
                file.write(line)
            for key in PodStatus.cmd_int:
                line = 'int_' + str(key) + '\t' + str(PodStatus.cmd_int[str(key)]) + '\t\t' + str(round(clock(),2)) + '\n'
                file.write(line)

            ### Log pod state variables
            line = 'state' + '\t' + str(PodStatus.state) + '\t' + str(0) + '\t' + str(round(clock(),2)) + '\n' \
                    + 'spacex_state' + '\t' + str(PodStatus.spacex_state) + '\t' + str(0) + '\t' + str(round(clock(),2)) + '\n' \
                    + 'total_faults' + '\t' + str(PodStatus.total_faults) + '\t' + str(0) + '\t' + str(round(clock(),2)) + '\n' \
                    + 'throttle' + '\t' + str(PodStatus.throttle) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'D' + '\t' + str(PodStatus.true_data['D']['val']) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'V' + '\t' + str(PodStatus.true_data['V']['val']) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'A' + '\t' + str(PodStatus.true_data['A']['val']) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'A_std_dev' + '\t' + str(PodStatus.true_data['A']['std_dev']) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'A_filter_val' + '\t' + str(PodStatus.sensor_filter['IMU1_Z']['val']) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'Clock_interval' + '\t' + str(PodStatus.poll_interval) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'Brakes' + '\t' + str(int(PodStatus.Brakes)) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'HV' + '\t' + str(int(PodStatus.HV)) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n' \
                    + 'Vent_Sol' + '\t' + str(int(PodStatus.Vent_Sol)) + '\t' + str(0) + '\t' + str(round(clock(), 2)) + '\n'\
                    + 'stripe_count' + '\t' + str(PodStatus.true_data['stripe_count']) + '\t' + str(0) + '\t' + str(round(clock(),2)) + '\n'
            file.write(line)

        PodStatus.log_lastwrite = clock()


if __name__ == "__main__":

    PodStatus = Status()

    gui = '2'
    # print('Which GUI should I use?\n')
    # print('\t1\tConsole')
    # print('\t2\tExternal')
    # while gui != '1' and gui != '2':
    #     gui = str(input('Enter choice: '))

    init()

    if PodStatus.init is False:
        PodStatus.Quit = True
        print("Failed to init.")

    while PodStatus.Quit is False:
        write_file()
        poll_sensors()
        filter_data()
        sensor_fusion()
        run_state()
        do_commands()
        eval_abort()
        rec_data()
        spacex_data()
        client.send_message(*addr, 'send_data', PodStatus.data_dump())

    # DEBUG...REMOVE BEFORE FLIGHT
    print("Quitting")

