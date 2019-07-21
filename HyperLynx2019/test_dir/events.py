from time import sleep

def Safing(object, **kwargs):
    # autosequence
    print('{} is safing.'.format(object.name))

    # transitions
    object.pod_safe()
    sleep(1)

def SafeToApproach(object, **kwargs):
    # autosequence
    print('{} is now safe to approach.'.format(object.name))

    # transitions
    object.cmd_launch()
    sleep(1)

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


