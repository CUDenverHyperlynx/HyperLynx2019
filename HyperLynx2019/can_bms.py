#!/usr/bin/python3
#
# simple_rx_test.py
#
# This is simple CAN receive python program. All messages received are printed out on screen.
# For use with PiCAN boards on the Raspberry Pi
# http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-2-p-1475.html
#
# Make sure Python-CAN is installed first http://skpang.co.uk/blog/archives/1220
#
# 01-02-16 SK Pang
#
# https://github.com/skpang/PiCAN-Python-examples/blob/master/simple_rx_test.py
#

import can
import time
import os

def run(object):
    print('\n\rCAN Rx test')
    print('Bring up CAN0....')
    os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    time.sleep(0.1)

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        print('Cannot find PiCAN board.')
        exit()

    print('Ready')

    datamap = {0x6B0:{'name':'PopulatedCells', 'val_float':0, 'mult':1},
               0x6B1:{'name':'PackSumVoltage', 'val_float':0, 'mult':10},
               0x6B2:{'name':'AvgCellVoltage', 'val_float':0, 'mult':10000},
               0x6B3:{'name':'FailsafeStatus', 'val_float':0, 'mult':1, 'val_str':'None'},
               }
    failsafemap = {0x00:'No failsafe active',
                   0x01:'Voltage failsafe active',
                   0x02:'Current failsafe active',
                   0x04:'Relay failsafe active',
                   0x08:'Cell balancing active (non-failsafe mode)',
                   0x10:'Charge interlock failsafe active',
                   0x20:'Thermistor B-value table invalid',
                   0x40:'Input power supply failsafe active'
                   }

    try:
        while True:
            message = bus.recv()  # Wait until a message is received.

            c = '{0:f} {1:x} {2:x} '.format(message.timestamp, message.arbitration_id, message.dlc)
            arb_ID = message.arbitration_id
            s = ''
            for i in range(message.dlc):
                s += '{0:x} '.format(message.data[i])

            print(' {}'.format(c + s))
            value = float(s,16)
            for key in datamap:
                if arb_ID == key:
                    datamap[key]['val_float'] = float(s) * datamap[key]['mult']
                    print(datamap[0x6B3])
                    if arb_ID == 0x6B3:
                        datamap[0x6B3]['val_str'] = failsafemap[s]



    except KeyboardInterrupt:
        # Catch keyboard interrupt
        os.system("sudo /sbin/ip link set can0 down")
        print('\n\rKeyboard interrtupt')

    return(datamap)