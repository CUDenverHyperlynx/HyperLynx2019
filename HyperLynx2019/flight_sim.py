"""
Flight simulator
    Pod sends current state and variables each loop.  sim() returns new
    values for this data based on pod's current state.

    Does not currently feature protections against re-using Reservoir air charges.
"""

#from time import clock
import random, numpy


def sim(PodStatus):

    print("In Flight Simulation")
    PodStatus.throttle += 0.1

    # Activate Res1
    if (PodStatus.cmd_ext['Res1_Sol'] == True) or (PodStatus.cmd_int['Res1_Sol'] == True) and (PodStatus.Vent_Sol == True):
        PodStatus.sensor_data['Brake_Pressure'] = PodStatus.sensor_data['Brake_Pressure'] + \
                                                  200 + random.randint(-10,10)*10**-2
    # Activate Res2
    if (PodStatus.cmd_ext['Res2_Sol'] == True) or (PodStatus.cmd_int['Res2_Sol'] == True) and (PodStatus.Vent_Sol == True):
        PodStatus.sensor_data['Brake_Pressure'] = PodStatus.sensor_data['Brake_Pressure'] + \
                                                  200 + random.randint(-10,10)*10**-2
    # Activate Vent
    if (PodStatus.cmd_ext['Vent_Sol'] == False) or (PodStatus.cmd_int['Vent_Sol'] == False):
        PodStatus.sensor_data['Brake_Pressure'] = 0.01 + random.randint(-10,10)*10**-3

    if PodStatus.Brakes is False:

        # Increment accelerometer data
        PodStatus.sensor_data['IMU1_X'] = PodStatus.throttle * 0.7 + random.randint(-10,10)*10**-2
        PodStatus.sensor_data['IMU2_X'] = PodStatus.throttle * 0.7 + random.randint(-10,10)*10**-2
        print(str(PodStatus.sensor_data['IMU1_X']))
        print(str(PodStatus.sensor_data['IMU2_X']))

        # Increment motor resolver data
        PodStatus.sensor_data['SD_MotorData_MotorRPM'] = (PodStatus.true_data['V']['val'] + \
                                                         PodStatus.true_data['A']['val'] * PodStatus.poll_interval + \
                                                         random.randint(-10,10)*10**-2) * 60 / PodStatus.wheel_circum

        # Increment stripe count
            # Ensure stripe count only incremented when:
            #   - current distance is > 25ft from the previous count point
            #   - current distance is < 5ft from the next stripe
        if (PodStatus.true_data['D']['val']/100 - PodStatus.stripe_count > 25) and \
                (abs(PodStatus.true_data['D']['val']/100 - numpy.around(PodStatus.true_data['D']['val']/100)) < 0.05):
            PodStatus.sensor_data['LST_Right'] += 1
            PodStatus.sensor_data['LST_Left'] += 1

    print("D:" + str(PodStatus.true_data['D']['val']) + '\t' + "V:" + str(PodStatus.true_data['V']['val']) + '\t' + "A:" + str(PodStatus.true_data['A']['val']))
    print("Left Stripe:" + str(PodStatus.sensor_data['LST_Left']) + '\t' + "Right Stripe:" + str(PodStatus.sensor_data['LST_Right']))

    return(PodStatus)


