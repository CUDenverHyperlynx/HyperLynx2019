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

def load_abort_ranges(file):
    df = pd.read_csv(file, sep='\t')


def clean(df):
    HV = df.Label == 'HV'
    df.loc[~HV, 'Value'] = pd.to_numeric(df.loc[~HV, 'Value'])

def serve_data(df):
    steps = df.Time.unique()
    return [df.loc[df.Time==step,:] for step in steps]


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Pod Data Simulator')
    parser.add_argument('--log', help='path to log file')
    args = parser.parse_args()

    print(args.log)
