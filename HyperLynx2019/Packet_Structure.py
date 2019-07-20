from time import clock
import pandas as pd


class Status:
    # Definition of State Numbers
    SafeToApproach = 1
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7

    # Init dictionaries
    abort_ranges = {SafeToApproach: {},
                    Launching: {},
                    BrakingHigh: {},
                    Crawling: {},
                    BrakingLow: {}}
    commands = {}               # Contains all possible inputs from GUI
    sensor_data = {}            # Contains all inputs from I2C/CAN buses
    sensor_filter = {}
    true_data = {}

    poll_oldtime = 0    # Vars for poll_sensors for integrating distance from speed
    poll_newtime = 0
    poll_interval = 0

    def __init__(self):        # BOOT INIT
        self.init = False
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
        # self.IMU1_addr = 0x28
        # self.IMU2_addr = 0x29
        # self.IMU1 = BNO055(None, self.IMU1_addr)
        # self.IMU2 = BNO055.BNO055(None, self.IMU2_addr)
        # self.IMU1.begin()
        # self.IMU2.begin()

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

    def load_abort_ranges(self, file):
        df = pd.read_csv(file, sep='\t').set_index('Sensor')
        df.columns = [col.strip() for col in df.columns]
        df.rename(columns={'Fault (INIT TO ZERO)':'Fault'}, inplace=True)
        values = ['Low', 'High', 'Trigger', 'Fault']
        states = {
            self.SafeToApproach: df.loc[df['1 - S2A'] == 1, values].to_dict('index'),
            self.Launching: df.loc[df['3 - Launch'] == 1, values].to_dict('index'),
            self.BrakingHigh:df.loc[df['5 - Brake1'] == 1, values].to_dict('index'),
            self.Crawling: df.loc[df['6 - Crawling'] == 1, values].to_dict('index'),
            self.BrakingLow:df.loc[df['7 - Brake2'] == 1, values].to_dict('index')}
        self.abort_ranges.update(states)

    # debug
    sensor_data['Brake_Pressure'] = 178
