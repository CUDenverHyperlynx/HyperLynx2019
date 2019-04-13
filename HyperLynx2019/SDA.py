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

#from argparse import ArgumentParser
from time import sleep, clock
import socket, struct
import numpy
import datetime
import os, psutil
#import smbus
import Hyperlynx_ECS, flight_sim
from Client import send_server

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
    sensor_filter = {}
    true_data = {}

    poll_oldtime = 0    # Vars for poll_sensors for integrating distance from speed
    poll_newtime = 0
    poll_interval = 0

    def __init__(self):        # BOOT INIT
        self.init = False
        self.flight_sim = False
        self.wheel_diameter = 17.4 / 12 # [ft] define drive wheel diameter
        self.wheel_circum = numpy.pi * self.wheel_diameter
        self.StartTime = clock()
        self.HV = 0                     # Current state of HV system (1 or 0)
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

        self.filter_length = 20         # Moving average for sensor data filter

        # SPACEX CONFIG DATA
        self.spacex_state = 0
        self.spacex_team_id = 69
        self.spacex_server_ip = '192.168.0.1'
        self.spacex_server_port = 3000
        self.spacex_rate = 40               # [Hz] rate of spacex data burst
        self.spacex_lastsend = 0

        # I2C init
        self.IMU_init_range = 0.001
        self.sensor_poll = Hyperlynx_ECS.HyperlynxECS()
        self.sensor_poll.initializeSensors()


        # DEBUG init for script:
        self.Quit = False

        # Set filter on priority data:
        self.filter_items = ['IMU1_X', 'IMU1_Y', 'IMU1_Z', 'IMU2_X', 'IMU2_Y',
                             'IMU2_Z', 'LIDAR', 'Brake_Pressure',
                             'SD_MotorData_MotorRPM']

        # init True values for Distance, Velocity, and Acceleration, with moving average queue, true value, and dev
        self.true_data = {'D': {'q': [], 'val': 0, 'std_dev': 0},
                          'V': {'q': [], 'val': 0, 'std_dev': 0},
                          'A': {'q': [], 'val': 0, 'std_dev': 0}}

        # Pod Abort conditions init:
        self.Fault = False
        self.Abort = False

        # INITIATE STATE TO S2A
        self.state = self.SafeToApproach

        # INITIATE LOG RATE INFO
        self.log_lastwrite = clock()            # Saves last time of file write to control log rate
        self.log_rate = 10                      # Hz


def init():
    # Create Abort Range and init sensor_data Dictionary from template file
    abort_names = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                   dtype=str)
    abort_vals = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 12))
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

    # Create Commands Dictionary from template file
    cmd_names = numpy.genfromtxt('commands.txt', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                 dtype=str)
    cmd_vals = numpy.genfromtxt('commands.txt', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 2),
                                dtype=int)
    for i in range(0, len(cmd_names)):
        PodStatus.commands[cmd_names[i]] = cmd_vals[i]

    cmd_int = dict()
    cmd_ext = dict()
    cmd_intval = dict()
    cmd_extval = dict()

    cmd_int = numpy.genfromtxt('cmd_int', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                               dtype=str)
    cmd_ext = numpy.genfromtxt('cmd_ext', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                               dtype=str)
    cmd_intval = numpy.genfromtxt('cmd_int', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 2),
                                  dtype=int)
    cmd_extval = numpy.genfromtxt('cmd_ext', skip_header=1, delimiter='\t', usecols=numpy.arange(1, 2),
                                  dtype=int)

    print("Checking IMUs")
    poll_sensors()
    filter_data()

    ## CHECK IMU INIT

    if abs(PodStatus.sensor_filter['IMU1_X']['val']) < PodStatus.IMU_init_range and \
            abs(PodStatus.sensor_filter['IMU2_X']['val']) < PodStatus.IMU_init_range:
        print("Both IMUs valid.")
        PodStatus.init = True
    else:
        print("IMU init failed.")

    ### Create log file ###
    date = datetime.datetime.today()
    new_number = str(date.year) + str(date.month) + str(date.day) \
                 + str(date.hour) + str(date.minute) + str(date.second)
    PodStatus.file_name = 'log_' + new_number
    file = open(os.path.join('logs/', PodStatus.file_name), 'a')
    columns = ['Label','Value','Fault','Time']
    with file:
        file.write('\t'.join(map(lambda column_title: "\"" + column_title + "\"", columns)))
        file.write("\n")
    file.close()

    ## Confirm boot info ##
    print("Pod init complete, State: " + str(PodStatus.state))
    print("Log file created: " + str(PodStatus.file_name))

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
    if PodStatus.flight_sim == True:
        flight_sim.sim(PodStatus)
    else:
        #PodStatus.sensor_data['Brake_Pressure'] = PodStatus.sensor_poll.getBrakePressure()
        PodStatus.sensor_data['LVBatt_Temp'] = PodStatus.sensor_poll.getBatteryTemp()
        PodStatus.sensor_data['LVBatt_Current'] = PodStatus.sensor_poll.getCurrentLevel()
        PodStatus.sensor_data['LVBatt_Voltage'] = PodStatus.sensor_poll.getVoltageLevel()
        PodStatus.sensor_data['PV_Left_Temp'] = PodStatus.sensor_poll.getBMEtemperature(2)
        PodStatus.sensor_data['PV_Left_Pressure'] = PodStatus.sensor_poll.getBMEpressure(2)
        PodStatus.sensor_data['PV_Right_Temp'] = PodStatus.sensor_poll.getBMEtemperature(1)
        PodStatus.sensor_data['PV_Right_Pressure'] = PodStatus.sensor_poll.getBMEpressure(1)
        PodStatus.sensor_data['Ambient_Pressure'] = PodStatus.sensor_poll.getTubePressure()
        PodStatus.sensor_data['IMU1_X'] = PodStatus.sensor_poll.getOrientation(1)[0]
        PodStatus.sensor_data['IMU1_Y'] = PodStatus.sensor_poll.getOrientation(1)[1]
        PodStatus.sensor_data['IMU1_Z'] = PodStatus.sensor_poll.getOrientation(1)[2]
        PodStatus.sensor_data['IMU2_X'] = PodStatus.sensor_poll.getOrientation(2)[0]
        PodStatus.sensor_data['IMU2_Y'] = PodStatus.sensor_poll.getOrientation(2)[1]
        PodStatus.sensor_data['IMU1_Z'] = PodStatus.sensor_poll.getOrientation(2)[2]
        PodStatus.sensor_data['LIDAR'] = PodStatus.sensor_poll.getLidarDistance()


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
    # Set pod state variable for brakes

    PodStatus.sensor_data['Brake_Pressure'] = 178
    if PodStatus.sensor_data['Brake_Pressure'] > 177:
        PodStatus.Brakes = False
    else:
        PodStatus.Brakes = True

    # Set pod state variable for speed and acceleration
    old_speed = PodStatus.speed
    if PodStatus.sensor_data['SD_MotorData_MotorRPM']:
        PodStatus.speed = PodStatus.sensor_data['SD_MotorData_MotorRPM'] / 60 * 2 * numpy.pi * PodStatus.wheel_diameter
    PodStatus.accel = numpy.mean([PodStatus.sensor_data['IMU1_X'], PodStatus.sensor_data['IMU2_X']])

    # Integrate distance.  At 1.4GHz clock speeds, integration can be numerically
    # approximated as constant speed over the time step.
    if PodStatus.poll_oldtime != 0:         # Update distance (but skip init loop @ old_time = 0
        PodStatus.distance += PodStatus.speed * PodStatus.poll_interval

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
                PodStatus.sensor_filter[str(key)]['std_dev'] = numpy.std(PodStatus.sensor_filter[str(key)]['q'])
                PodStatus.sensor_filter[str(key)]['mean'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])

                # if new value is inside range of std_dev (hence valid), then add to q
                if abs(PodStatus.sensor_data[str(key)]-PodStatus.sensor_filter[str(key)]['mean']) <= \
                        PodStatus.sensor_filter[str(key)]['std_dev']:

                    # shift q values over
                    for i in range(0, (len(PodStatus.sensor_filter[str(key)]['q'])-1)):
                        PodStatus.sensor_filter[str(key)]['q'][i] = PodStatus.sensor_filter[str(key)]['q'][i+1]
                    # add new value to end of queue
                    PodStatus.sensor_filter[str(key)]['q'][(PodStatus.filter_length-1)] = PodStatus.sensor_data[str(key)]
                    # set the filtered value to the mean of the new queue
                    PodStatus.sensor_filter[str(key)]['val'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])

def sensor_fusion():
    """ Combines various filtered sensor data to a common solution."""

    ### BEGIN ACCELERATION FUSION
    # If queue is not full, fill queue
    good_IMUs = []  # reset good_IMUs to empty set
    if len(PodStatus.true_data['A']['q']) < PodStatus.filter_length:
        # Add mean of IMU values to
        PodStatus.true_data['A']['q'] = numpy.append(PodStatus.true_data['A']['q'],
                                        numpy.mean([PodStatus.sensor_filter['IMU1_X']['val'],
                                                    PodStatus.sensor_filter['IMU2_X']['val']]))
    else:
        PodStatus.true_data['A']['std_dev'] = numpy.std(PodStatus.true_data['A']['q'])

        # determine valid IMU data
        # if new sensor_filter data is within 2*std_dev of true q, include in good_IMUs calc
        if abs(PodStatus.sensor_filter['IMU1_X']['val']-numpy.mean(PodStatus.true_data['A']['q'])) < \
                2 * PodStatus.true_data['A']['std_dev']:
            numpy.append(good_IMUs, PodStatus.sensor_filter['IMU1_X']['val'])
        if abs(PodStatus.sensor_filter['IMU2_X']['val'] - numpy.mean(PodStatus.true_data['A']['q'])) < \
                2 * PodStatus.true_data['A']['std_dev']:
            numpy.append(good_IMUs, PodStatus.sensor_filter['IMU2_X']['val'])

        # if good_IMUs is not empty set, take mean value and add to true_data q
        if good_IMUs:
            print("Good data found for A")
            # reset IMU_bad_time timer
            PodStatus.IMU_bad_time_elapsed = 0
            PodStatus.IMU_bad_time = None

            # Set true value to mean of good_IMUs array
            PodStatus.true_data['A']['val'] = numpy.mean(good_IMUs)

            # add valid data to true_data q
            # shift q values over
            for i in range(0, (len(PodStatus.true_data['A']['q']) - 1)):
                PodStatus.true_data['A']['q'][i] = PodStatus.true_data['A']['q'][i + 1]
            # add new value to end of queue
            PodStatus.true_data['A']['q'][(PodStatus.filter_length - 1)] = PodStatus.true_data['A']['val']

        # if no good IMU data, start or progress the bad IMU data timer
        else:
            if not PodStatus.IMU_bad_time:
                PodStatus.IMU_bad_time = clock()
                print("Bad IMU data, starting 1 second clock at " + str(PodStatus.IMU_bad_time))

            else:
                PodStatus.IMU_bad_time_elapsed = clock()-PodStatus.IMU_bad_time
                print("Bad IMU data, elapsed time: " + str(PodStatus.IMU_bad_time_elapsed))


    ### END ACCELERATION FUSION

    ### BEGIN VELOCITY FUSION
    # If queue is not full, fill queue
    if len(PodStatus.true_data['V']['q']) < PodStatus.filter_length:
        # Add mean of IMU values to
        PodStatus.true_data['V']['q'] = numpy.append(PodStatus.true_data['V']['q'],
                                                     PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val'])

    else:
        # Estimate new velocity
        Vdr = PodStatus.poll_interval * PodStatus.true_data['A']['val']

        # Set std_dev
        PodStatus.true_data['V']['std_dev'] = numpy.std(PodStatus.true_data['V']['q'])

        # Evaluate new data compared to true q and std dev
        if abs(PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val'] - numpy.mean(PodStatus.true_data['V']['q'])) < \
                2 * PodStatus.true_data['V']['std_dev']:
            PodStatus.V_bad_time_elapsed = 0
            PodStatus.V_bad_time = None
            PodStatus.true_data['V']['val'] = PodStatus.sensor_filter['SD_MotorData_MotorRPM']['val']

        # if new data is invalid:
        else:
            # RUN TRACTION CONTROL FUNCTION

            # Start or progress bad V data timer
            if not PodStatus.V_bad_time:
                PodStatus.V_bad_time = clock()
                print("Bad V data, starting clock at " + str(PodStatus.IMU_bad_time))

            else:
                PodStatus.V_bad_time_elapsed = clock()-PodStatus.V_bad_time
                print("Bad V data, elapsed time: " + str(PodStatus.V_bad_time_elapsed))
    ### END VELOCITY FUSION

    ### BEGIN DISTANCE FUSION
    PodStatus.true_data['D']['val'] = PodStatus.poll_interval * \
        PodStatus.true_data['V']['val']

    PodStatus.true_data['stripe_dist'] = numpy.maximum(PodStatus.sensor_data['LST_Left'],
                                                    PodStatus.sensor_data['LST_Right'])

    PodStatus.D_diff = PodStatus.true_data['D']['val'] - PodStatus.true_data['stripe_dist']

    if PodStatus.true_data['D']['val'] > 25:
        temp_count = PodStatus.true_data['D']['val'] / 100
        temp_dec = abs(temp_count - numpy.around(temp_count))
        if temp_dec < 0.2 and PodStatus.D_diff > 20:
            print("Looking for stripe")
            if PodStatus.sensor_data['LST_Left'] == temp_count:
                print("Left stripe counted!")
                PodStatus.true_data['stripe_dist'] = PodStatus.sensor_data['LST_Left']
                PodStatus.sensor_data['LST_Right'] = PodStatus.sensor_data['LST_Left']
            elif PodStatus.sensor_data['LST_Right'] == temp_count:
                print("Right stripe counted!")
                PodStatus.true_data['stripe_dist'] = PodStatus.sensor_data['LST_Right']
                PodStatus.sensor_data['LST_Left'] = PodStatus.sensor_data['LST_Right']
            else:
                print("No stripe counted, still waiting.")

        else:
            pass


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

    if PodStatus.Abort is True:
        PodStatus.commands['Abort'] = 1
        abort()

def rec_data():
    """
    This function receives data from the GUI, primarily commands[]
    During DEBUG, this function is a mock GUI for testing SDA.
    """

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
        "*************************")

    if PodStatus.state == PodStatus.SafeToApproach:
        print("\n*** MENU ***\n"
              "\t(L) Launch\n"
              "\t(R) Reset Abort Flag\n"
              "\t(Q) Quit\n"
              "\t(FS) Activate Flight Sim\n"
              "\t\tType line number or letter command to change values.\n")
        print(str(PodStatus.sensor_data['Brake_Pressure']))
        a = input('Enter choice: ')
        if a == '2':
            if PodStatus.HV is False:
                PodStatus.commands['HV'] = True
                PodStatus.HV = True
            else:
                PodStatus.commands['HV'] = False
                PodStatus.HV = False
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
            PodStatus.commands['para_BBP'] = float(input("Enter BBP Distance in feet: "))
        elif a == '8':
            PodStatus.commands['para_max_speed'] = float(input("Enter max speed in ft/s: "))
        elif a == '9':
            PodStatus.commands['para_max_accel'] = float(input("Enter max accel in g: "))
        elif a == '10':
            PodStatus.commands['para_max_time'] = float(input("Enter max time in s: "))
        elif a == '11':
            PodStatus.commands['para_max_crawl_speed'] = float(input("Enter max crawl speed in ft/s: "))
        elif a == 'L':
            PodStatus.commands['Launch'] = 1
        elif a == 'R':
            PodStatus.commands['Abort'] = 0
            PodStatus.stopped_time = -1
            PodStatus.MET_startime = -1
        elif a == 'Q':
            PodStatus.Quit = True
        elif a == 'FS':
            PodStatus.flight_sim = True
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
        elif PodStatus.commands['Abort'] == 0 and PodStatus.Abort is True:
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
        if PodStatus.commands['para_max_crawl_speed'] != PodStatus.para_max_crawl_speed:
            PodStatus.para_max_crawl_speed = PodStatus.commands['para_max_crawl_speed']

    else:
        # Load ONLY abort command
        if PodStatus.commands['Abort'] is True:
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

def send_data(pod_status):        # Sends data to TCP (GUI) and CAN (BMS/MC)

    ### Send to CAN ###

    ### Send to TCP ###

    send_server(pod_status)

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
        elif PodStatus.speed > PodStatus.para_max_speed:
            print("Pod has reached max speed.")
            transition()
        elif PodStatus.MET > PodStatus.para_max_time:
            print("Pod has exceeded max time.")
            transition()

        # TRANSITIONS FOR BAD DATA
        elif PodStatus.abort_ranges[PodStatus.state]['IMU_bad_time_elapsed']['Fault'] == 1:
            print("Transition for bad IMU data.")
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

        if PodStatus.speed <= 0.5 and PodStatus.stopped_time <= 0:
            PodStatus.stopped_time = clock()

        if PodStatus.speed > 0.5:
                                        # THIS VALUE NEEDS TO BE THOROUGHLY TESTED;
                                        # IF ERRANT SPEED VALUES > 0.5 WHILE ACTUALLY
                                        # STOPPED, COULD CAUSE EXCESSIVE DISCHARGE
                                        # OF RESERVOIRS AND LOSS OF BRAKE RETRACTION
                                        # ABILITY
            PodStatus.stopped_time = 0              # RESET STOPPED TIME
            if PodStatus.commands['Vent_Sol'] == 1:    # OPEN BRAKE VENT SOLENOID
                print("Opening Vent Sol")
                PodStatus.commands['Vent_Sol'] = 0
            if PodStatus.throttle > 0:          # SET THROTTLE TO 0
                PodStatus.throttle = 0

        # DO NOTHING ELSE UNTIL STOPPED
        # RECONFIGURE FOR CRAWLING

        ## RECONFIGURE FOR CRAWLING STATE

        if PodStatus.speed < 0.5 and (clock() - PodStatus.stopped_time) > 5:
            if PodStatus.commands['Vent_Sol'] == 0:
                PodStatus.commands['Vent_Sol'] = 1     # CLOSE BRAKE VENT SOLENOID
                print("Closing Vent Sol")
            if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] < 20:
                PodStatus.commands['Res1_Sol'] = 1     # OPEN RES#1 SOLENOID
                print("Opening Res#1, pausing for 2 seconds.")

            else:
                print("Waiting for pod to achieve braking pressure")
                print("Vent_Sol: " + str(PodStatus.Vent_Sol))
                print("Res1_Sol: " + str(PodStatus.Res1_Sol))
                print("Brake Pressure: " + str(PodStatus.sensor_data['Brake_Pressure']))

            if PodStatus.Vent_Sol == 1 and PodStatus.sensor_data['Brake_Pressure'] > 177:
                print("Brakes retracted, closing Res1 solenoid.")
                PodStatus.commands['Res1_Sol'] = 0  # CLOSE RES#1 SOLENOID
                transition()


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
        print("Abort flagged in S2A.")

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
                    + 'accel' + '\t' + str(PodStatus.accel) + '\t\t' + str(round(clock(), 2)) + '\n' \
                    + 'wheel_diameter' + '\t' + str(PodStatus.wheel_diameter) + '\t\t' + str(round(clock(),2)) + '\n'
            file.write(line)

        PodStatus.log_lastwrite = clock()

if __name__ == "__main__":

    PodStatus = Status()
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
        send_data(PodStatus)
        print(PodStatus.sensor_data['IMU1_X'])

    # DEBUG...REMOVE BEFORE FLIGHT
    print("Quitting")

