## test out the sensor polling

import numpy
import time
class Status:

    sensor_filter = {'Brake_Pressure': {'q':[]}}
    sensor_data = {}
    filter_length = 20
    std_dev_mult = 1.5

def poll_sensors():
    for key in PodStatus.sensor_filter:

        # If queue is not full, fill queue
        if len(PodStatus.sensor_filter[str(key)]['q']) < PodStatus.filter_length:
            PodStatus.sensor_filter[str(key)]['q'] = numpy.append(PodStatus.sensor_filter[str(key)]['q'],
                                                                  PodStatus.sensor_data[str(key)])
        # If queue is full, do this:
        if len(PodStatus.sensor_filter[str(key)]['q']) == PodStatus.filter_length:
            PodStatus.sensor_filter[str(key)]['std_dev'] = numpy.std(PodStatus.sensor_filter[str(key)]['q'])
            PodStatus.sensor_filter[str(key)]['mean'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])

            # if new value is inside range of std_dev multiple(hence valid), then add to q
            if abs(PodStatus.sensor_data[str(key)] - PodStatus.sensor_filter[str(key)]['mean']) <= \
                    PodStatus.std_dev_mult * PodStatus.sensor_filter[str(key)]['std_dev']:
                print("\tValid data.")
                # shift q values over
                for i in range(0, (len(PodStatus.sensor_filter[str(key)]['q']) - 1)):
                    PodStatus.sensor_filter[str(key)]['q'][i] = PodStatus.sensor_filter[str(key)]['q'][i + 1]
                # add new value to end of queue
                PodStatus.sensor_filter[str(key)]['q'][(PodStatus.filter_length - 1)] = PodStatus.sensor_data[str(key)]
            else:
                print("NOT Valid data, skipping.")

        # set the filtered value to the mean of the new queue
        PodStatus.sensor_filter[str(key)]['val'] = numpy.mean(PodStatus.sensor_filter[str(key)]['q'])


if __name__ == '__main__':
    PodStatus = Status()

    while True:
        PodStatus.sensor_data['Brake_Pressure'] = 1000 + numpy.random.randint(-10, 21)
        print("\nNew data point: " + str(PodStatus.sensor_data['Brake_Pressure']))
        poll_sensors()
        if len(PodStatus.sensor_filter['Brake_Pressure']['q']) == PodStatus.filter_length:
            #print("Std dev: " + str(PodStatus.sensor_filter['Brake_Pressure']['std_dev']))
            print("Q: " + str(PodStatus.sensor_filter['Brake_Pressure']['q']))
            print("Val: " + str(PodStatus.sensor_filter['Brake_Pressure']['val']))
            time.sleep(1)
