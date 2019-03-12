## test out the sensor polling

import numpy as np
import time
class Status:
    poll_raw = {'Brake_Pressure' : 0}
    poll_raw_q = {'Brake_Pressure' : []}
    poll_filter = {'Brake_Pressure' : []}
    moving_avg_count = 20
    loop_speed = 0.000020

def poll_sensors():

    PodStatus.poll_raw['Brake_Pressure'] = 1000 + np.random.randint(-10,21)

    for key in PodStatus.poll_raw_q:
        #print(str(len(PodStatus.poll_raw_q[str(key)])))
        if len(PodStatus.poll_raw_q[str(key)]) < PodStatus.moving_avg_count:
            #print("Adding to q\n")
            PodStatus.poll_raw_q[str(key)] = np.append(PodStatus.poll_raw_q[str(key)], PodStatus.poll_raw[str(key)])
        else:
            #print("Shifting q\n")
            for i in range(0, (len(PodStatus.poll_raw_q[str(key)])-1)):
                PodStatus.poll_raw_q[str(key)][i] = PodStatus.poll_raw_q[str(key)][i+1];
            PodStatus.poll_raw_q[str(key)][(PodStatus.moving_avg_count-1)] = PodStatus.poll_raw[str(key)];
            np.append(PodStatus.poll_filter[str(key)], np.average(PodStatus.poll_raw_q[str(key)]))


if __name__ == '__main__':
    PodStatus = Status()
    timer = time.clock()
    while True:
        poll_sensors()
        print(PodStatus.poll_raw_q['Brake_Pressure'])
        length = len(PodStatus.poll_filter['Brake_Pressure'])
        if length == PodStatus.moving_avg_count:
            print(str(round(PodStatus.poll_filter['Brake_Pressure'][length],3)))
        std_dev = np.std(PodStatus.poll_raw_q['Brake_Pressure'])
        error = std_dev / PodStatus.poll_filter['Brake_Pressure']
        print("Std dev: " + str(std_dev))
        print("% error: " + str(100*error))
        print("Total Time: " + str(time.clock()-timer))