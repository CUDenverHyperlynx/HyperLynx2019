## test out the sensor polling
import numpy as np
import time
class Status:
    poll_raw = {'Brake_Pressure' : 0}
    poll_raw_q = {'Brake_Pressure' : []}
    poll_filter = {'Brake_Pressure' : 0}
    moving_avg_count = 20

def poll_sensors():

    PodStatus.poll_raw['Brake_Pressure'] = np.random.rand()

    for key in PodStatus.poll_raw_q:
        print(str(len(PodStatus.poll_raw_q[str(key)])))
        if len(PodStatus.poll_raw_q[str(key)]) < PodStatus.moving_avg_count:
            print("Adding to q\n")
            PodStatus.poll_raw_q[str(key)] = np.append(PodStatus.poll_raw_q[str(key)], PodStatus.poll_raw[str(key)])
        else:
            print("Shifting q\n")
            for i in range(0, (len(PodStatus.poll_raw_q[str(key)])-1)):
                PodStatus.poll_raw_q[str(key)][i] = PodStatus.poll_raw_q[str(key)][i+1];
            PodStatus.poll_raw_q[str(key)][(PodStatus.moving_avg_count-1)] = PodStatus.poll_raw[str(key)];
            PodStatus.poll_filter[str(key)] = np.average(PodStatus.poll_raw_q[str(key)])


if __name__ == '__main__':
    PodStatus = Status()
    timer = time.clock()
    while True:
        poll_sensors()
        print(PodStatus.poll_filter['Brake_Pressure'])
        time.sleep(0.05)
        if time.clock()-timer > 5:
            break