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

    # Activate Res1
    if (PodStatus.cmd_ext['Res1_Sol'] == 1) or (PodStatus.cmd_int['Res1_Sol'] == 1) and (PodStatus.Vent_Sol == 1):
        PodStatus.sensor_data['Brake_Pressure'] = 200 + random.randint(-10,10)*10**-2

    # Activate Res2
    if (PodStatus.cmd_ext['Res2_Sol'] == 1) or (PodStatus.cmd_int['Res2_Sol'] == 1) and (PodStatus.Vent_Sol == 1):
        PodStatus.sensor_data['Brake_Pressure'] = 200 + random.randint(-10,10)*10**-2

    # Activate Vent
    if (PodStatus.cmd_ext['Vent_Sol'] == 0) or (PodStatus.cmd_int['Vent_Sol'] == 0):
        PodStatus.sensor_data['Brake_Pressure'] = 0.01 + random.randint(-10,10)*10**-3

    if PodStatus.Brakes is False:

        # Increment accelerometer data
        PodStatus.sensor_data['IMU1_Z'] = PodStatus.throttle * PodStatus.para_max_accel + random.randint(-1,1)*10**-2
        PodStatus.sensor_data['IMU2_Z'] = PodStatus.throttle * PodStatus.para_max_accel + random.randint(-1,1)*10**-2
        print(str(PodStatus.sensor_data['IMU1_Z']))
        print(str(PodStatus.sensor_data['IMU2_Z']))

        # Increment motor resolver data
        PodStatus.sensor_data['SD_MotorData_MotorRPM'] = (PodStatus.true_data['V']['val'] + \
                                                         PodStatus.true_data['A']['val'] * PodStatus.poll_interval + \
                                                         random.randint(-1,1)*10**-2) * 60 / PodStatus.wheel_circum

        # Increment stripe count
            # Ensure stripe count only incremented when:
            #   - current distance is > 25ft from the previous count point
            #   - current distance is < 5ft from the next stripe
        if ((PodStatus.true_data['D']['val']/100 - PodStatus.stripe_count) > 25) and \
                (abs(PodStatus.true_data['D']['val']/100 - numpy.around(PodStatus.true_data['D']['val']/100)) < 0.05):
            PodStatus.sensor_data['LST_Right'] += 1
            PodStatus.sensor_data['LST_Left'] += 1

    if PodStatus.Brakes is True:

        # Increment accelerometer data
        PodStatus.sensor_data['IMU1_Z'] = -8 - random.randint(-1,1)*10**-2
        PodStatus.sensor_data['IMU2_Z'] = -8 - random.randint(-1,1)*10**-2

        # Increment motor resolver data
        PodStatus.sensor_data['SD_MotorData_MotorRPM'] = (PodStatus.true_data['V']['val'] + \
                                                         PodStatus.true_data['A']['val'] * PodStatus.poll_interval + \
                                                         random.randint(-1,1)*10**-2) * 60 / PodStatus.wheel_circum

        if ((PodStatus.true_data['D']['val']/100 - PodStatus.stripe_count) > 25) and \
                (abs(PodStatus.true_data['D']['val']/100 - numpy.around(PodStatus.true_data['D']['val']/100)) < 0.05):
            PodStatus.sensor_data['LST_Right'] += 1
            PodStatus.sensor_data['LST_Left'] += 1

    print("Left Stripe:" + str(PodStatus.sensor_data['LST_Left']) + '\t' + "Right Stripe:" + str(PodStatus.sensor_data['LST_Right']))

    return(PodStatus)


