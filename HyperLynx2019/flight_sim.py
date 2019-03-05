"""
Flight simulator, by Jeff Stanek
Each loop, this imports the log file and reads the last 200 entries into the data{} dict.
The move() function calculates new speed/accel/distance based on last logged data entry.
Writes this information to the flight_sim_data.dat file, which is then fed back to the SDA
as fake sensor data.
"""

from time import clock
import os, numpy

class Struct():
    torque = 240 * 0.9   # N-m @ 90% efficiency
    g = 9.81
    mass = 250/2.2
    data = {}
    poll_oldtime = float()
    poll_newtime = 0
    poll_interval = float()
    log_rate = 20
    log_lastwrite = 0
    file_name = 'logs/flight_sim_data.dat'
    log_file = ''

def import_data():
    names = numpy.genfromtxt(('logs/' + state.log_file), skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1),
                                   dtype=str)
    vals = numpy.genfromtxt(('logs/' + state.log_file), skip_header=1, delimiter='\t', usecols=numpy.arange(1, 2),
                                  dtype=float)
    for i in range(0, len(names)):
        state.data[str(names[i])] =  float(vals[i])
    #print(state.data)


def move():
    state.poll_oldtime = state.poll_newtime
    state.poll_newtime = clock()
    state.poll_interval = state.poll_newtime-state.poll_oldtime

    if state.data['Brake_Pressure'] < 177:
        state.brakes = (177 - state.data['Brake_Pressure']) / 177
    else:
        state.brakes = 0

    state.data['distance'] += state.data['speed'] * state.poll_interval
    state.data['speed'] += state.data['accel'] * state.poll_interval
    state.data['accel'] = (state.data['throttle'] * state.torque / state.data['wheel_diameter']
                           - state.brakes * (1700/2.2 ) ) / (state.mass * state.g)

    if state.data['Vent_Sol']== 1 and state.data['Res1_Sol'] == 1:
        state.data['Brake_Pressure'] = 200
    elif state.data['Vent_Sol'] == 1 and state.data['Res2_sol'] == 1:
        state.data['Brake_Pressure'] = 200
    elif state.data['Vent_Sol'] == 0:
        state.data['Brake_Pressure'] = 0

def write_data():

    if (clock() - state.log_lastwrite) < (1/state.log_rate):
        pass

    else:

        file = open(state.file_name,'a')
        with file:
            ### Log sensor_data
            for key in state.data:
                line = str(key) + '\t' + str(state.data[str(key)]) +'\t'+ str(round(clock(),2)) + '\n'
                file.write(line)




    pass
if __name__ == '__main__':
    state = Struct()

    state.log_file = input("Enter log file name: ")
    while True:
        import_data()
        move()
        write_data()
