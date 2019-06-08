# Author: Nathan James
# HyperLynx GUI data simulator


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

commands = []

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
    return [df.loc[df.Time==step,:] for step in steps]

# def df_to_dict(df):
#


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Pod Data Simulator')
    parser.add_argument('--log', help='path to log file')
    args = parser.parse_args()

    data  = load_data_log(args.log)
