# Jose Ortega
# HyperLynx TCP Server

import socket
import pickle
from time import clock

# set up connection
HOST = socket.gethostbyname(socket.gethostname())       # Change IP Address when using radios
PORT = 1028
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))

# Print Listening on HOST and PORT
print(HOST)
print(PORT)

# loop during run
while 1:
    # Receive new data
    data, address = s.recvfrom(50000)
    data_variable = pickle.loads(data)

    # Print all data that was in the log file to the console
    for key in data_variable.sensor_data:
        if str(key) in data_variable.abort_ranges[data_variable.state]:
            fault_code = data_variable.abort_ranges[data_variable.state][str(key)]['Fault']
        else:
            fault_code = 0

        print(str(key) + '\t' + str(data_variable.sensor_data[str(key)]) + '\t' +
              str(int(fault_code)) + '\t' + str(round(clock(), 2)))

    for key in data_variable.commands:
        print(str(key) + '\t' + str(data_variable.commands[str(key)]) + '\t\t' + str(round(clock(), 2)))

    print('state' + '\t' + str(data_variable.state) + '\t\t' + str(round(clock(), 2)) + '\n' + 'spacex_state' + '\t'
          + str(data_variable.spacex_state) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'total_faults' + '\t' + str(data_variable.total_faults) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'throttle' + '\t' + str(data_variable.throttle) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'distance' + '\t' + str(data_variable.distance) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'speed' + '\t' + str(data_variable.speed) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'accel' + '\t' + str(data_variable.accel) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'wheel_diameter' + '\t' + str(data_variable.wheel_diameter) + '\t\t' + str(round(clock(), 2)) + '\n')

    # Call GUI function and send data
    # Put GUI function here
