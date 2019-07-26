from time import sleep

def Safing(object, **kwargs):
    # autosequence
    print('auto: safing.')
    object.HV = False
    object.speed = 0
    return(object)

def SafeToApproach(object, **kwargs):
    # autosequence
    # No actions taken.
    print('Now safe to approach.')
    return(object)

def Prelaunch(object, **kwargs):
    # autosequence
    print('{} is preparing to launch.'.format(object.name))

    # transitions
    object.prelaunch_ok()
    sleep(1)

def Launching(object):
    # autosequence
    print('{} is launching!'.format(object.name))
    print('Now moving at {} ft/s.'.format(object.telem['speed']))

    # transitions
    pass


