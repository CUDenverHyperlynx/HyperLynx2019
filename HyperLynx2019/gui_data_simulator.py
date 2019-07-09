# Author: Nathan James
# HyperLynx GUI data simulator


import numpy as np
import pandas as pd
import argparse

col_to_state = {'1 - S2A': "SafeToApproach",
                '2 - FC2l': "",
                '3 - Launch': 'Launching',
                '5 - Brake1': 'BrakingHigh',
                'Postflight': "",
                '6 - Crawling': 'Crawling',
                '7 - Brake2': 'BarkingLow',
                }

SafeToApproach = 1
Launching = 3
BrakingHigh = 5
Crawling = 6
BrakingLow = 7

sensors = ['LVBatt_Temp', 'LVBatt_Current', 'LVBatt_Voltage', 'PV_Left_Temp',
           'PV_Left_Pressure', 'PV_Right_Temp', 'PV_Right_Pressure',
           'Ambient_Pressure', 'IMU1_X', 'IMU1_Y', 'IMU1_Z', 'IMU2_X', 'IMU2_Y',
           'IMU2_Z', 'LIDAR', 'RPi_Disk_Space_Free', 'RPi_Disk_Space_Used',
           'RPi_Proc_Load', 'RPi_Mem_Load', 'RPi_Mem_Free', 'RPi_Mem_Used',
           'Brake_Pressure', 'SD_MotorData_MotorRPM']

sensor_filters = []

sensor_true = []

command_lst = []

def load_abort_ranges(file):
    df = pd.read_csv(file, sep='\t').set_index('Sensor')
    df.columns = [col.strip() for col in df.columns]
    df.rename(columns={'Fault (INIT TO ZERO)':'Fault'}, inplace=True)
    values = ['Low', 'High', 'Trigger', 'Fault']
    states = {
        SafeToApproach: df.loc[df['1 - S2A'] == 1, values].to_dict('index'),
        Launching: df.loc[df['3 - Launch'] == 1, values].to_dict('index'),
        BrakingHigh:df.loc[df['5 - Brake1'] == 1, values].to_dict('index'),
        Crawling: df.loc[df['6 - Crawling'] == 1, values].to_dict('index'),
        BrakingLow:df.loc[df['7 - Brake2'] == 1, values].to_dict('index')}
    return states


def load_data_log(file):
    df = pd.read_csv(file, sep='\t')
    HV = df.Label == 'HV'
    df.loc[~HV, 'Value'] = pd.to_numeric(df.loc[~HV, 'Value'])
    return df

def serve_data(df):
    steps = df.Time.unique()
    return [df.loc[df.Time==step,:].set_index('Label') for step in steps]

def df_to_dict(df):
    '''
    This function takes a dataframe of a single timestep and loads the data
    into respective dictionaries
    '''
    D = ['D', 'distance']
    V = ['V', 'speed']
    A = ['A', 'accel']

    return {
        'sensor_data': df.loc[df.index.isin(sensors), 'Value'].to_dict(),
        'commands': df.loc[df.index.isin(command_lst), 'Value'].to_dict(),
        'state': df.loc['state', 'Value'].to_dict(),
        'spacex_state': df.loc['spacex_state', 'Value'].to_dict(),
        'throttle': df.loc['throttle', 'Value'].to_dict(),
        'D': df.loc[df.index.isin(D), 'Value'].to_dict(),
        'V': df.loc[df.index.isin(V), 'Value'].to_dict(),
        'A': df.loc[df.index.isin(A), 'Value'].to_dict()
    }


class DataSimulator():
    SafeToApproach = 1
    Launching = 3
    BrakingHigh = 5
    Crawling = 6
    BrakingLow = 7

    def __init__(self):
        self.sensors = [
            'LVBatt_Temp', 'LVBatt_Current', 'LVBatt_Voltage', 'PV_Left_Temp',
            'PV_Left_Pressure', 'PV_Right_Temp', 'PV_Right_Pressure',
            'Ambient_Pressure', 'IMU1_X', 'IMU1_Y', 'IMU1_Z', 'IMU2_X', 'IMU2_Y',
            'IMU2_Z', 'LIDAR', 'RPi_Disk_Space_Free', 'RPi_Disk_Space_Used',
            'RPi_Proc_Load', 'RPi_Mem_Load', 'RPi_Mem_Free', 'RPi_Mem_Used',
            'Brake_Pressure', 'SD_MotorData_MotorRPM'
        ]

        self.sensor_filters = []

        self.sensor_true = []

        self.commands = []

        self.sensor_data = {}
        self.sensor_filter = {}
        self.true_data = {}

        self.poll_oldtime = 0
        self.poll_newtime = 0
        self.poll_interval = 0

    def load_abort_ranges(self, file):
       df = pd.read_csv(file, sep='\t').set_index('Sensor')
       df.columns = [col.strip() for col in df.columns]
       df.rename(columns={'Fault (INIT TO ZERO)':'Fault'}, inplace=True)
       values = ['Low', 'High', 'Trigger', 'Fault']
       self.abort_ranges = {
           SafeToApproach: df.loc[df['1 - S2A'] == 1, values].to_dict('index'),
           Launching: df.loc[df['3 - Launch'] == 1, values].to_dict('index'),
           BrakingHigh:df.loc[df['5 - Brake1'] == 1, values].to_dict('index'),
           Crawling: df.loc[df['6 - Crawling'] == 1, values].to_dict('index'),
           BrakingLow:df.loc[df['7 - Brake2'] == 1, values].to_dict('index')}

    def load_data_log(self, file):
       df = pd.read_csv(file, sep='\t')
       HV = df.Label == 'HV'
       df.loc[~HV, 'Value'] = pd.to_numeric(df.loc[~HV, 'Value'])
       times = df.Time.unique()
       self.data_list = [df.loc[df.Time==time,:].set_index('Label') for time in times]

    def df_to_dict(self, df):
       '''
       This function takes a dataframe of a single timestep and loads the data
       into respective dictionaries
       '''
       D = ['D', 'distance']
       V = ['V', 'speed']
       A = ['A', 'accel']

       return {
           'sensor_data': df.loc[df.index.isin(sensors), 'Value'].to_dict(),
           'commands': df.loc[df.index.isin(command_lst), 'Value'].to_dict(),
           'state': self._dict_exceptions(df, 'state'),
           'spacex_state': self._dict_exceptions(df, 'spacex_state'),
           'total_faults': self._dict_exceptions(df, 'total_fault'),
           'throttle': self._dict_exceptions(df, 'throttle'),
           'D': df.loc[df.index.isin(D), 'Value'].to_dict(),
           'V': df.loc[df.index.isin(V), 'Value'].to_dict(),
           'A': df.loc[df.index.isin(A), 'Value'].to_dict()
       }

    def _dict_exceptions(self, df, key):
       try:
           return df.loc[key, 'Value']
       except KeyError:
           return np.nan

    def __iter__(self):
        self.it = iter(self.data_list)
        return self

    def __next__(self):
        return self.df_to_dict(next(self.it))




if __name__ == "__main__":
    # this needs to be run from Hyperlynx2019 to work
    from network_transfer.libclient import BaseClient
    import time

    parser = argparse.ArgumentParser(description='Pod Data Simulator')
    parser.add_argument('--log', help='path to log file')
    parser.add_argument('--server', help='<host>:<port>')
    args = parser.parse_args()


    sim = DataSimulator()
    sim.load_data_log(args.log)

    it = iter(sim)

    if args.server:
        host, port = args.server.split(':')
        port = int(port)
        client = BaseClient()
        for i in it:
            client.send_message(host, port, 'send_data', i)
            time.sleep(1)
    else:
        for i in it:
            print(i)
            time.sleep(1)
