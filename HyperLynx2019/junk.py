import numpy

class State():
    data = {}
    sensor_names = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1), dtype=str)
    sensor_vals = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1,12), dtype=int)
    for i in range(0,len(sensor_names)):
        data[sensor_names[i]] = {'Low': { sensor_vals[i,0]},
                                 'High': { sensor_vals[i,1]},
                                 'States': { sensor_vals[i,2],
                                              sensor_vals[i,3],
                                              sensor_vals[i,4],
                                              sensor_vals[i,5],
                                              sensor_vals[i,6],
                                              sensor_vals[i,7],
                                              sensor_vals[i,8]
                                              },
                                 'Trigger': { sensor_vals[i,9] },
                                 'Fault': {sensor_vals[i, 10]}
                                 }

PodStatus = State()
print(PodStatus.data['BMS_Cell_Temp'])