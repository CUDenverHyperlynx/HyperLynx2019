"""
Flight simulator
    Pod sends current state and variables each loop.  sim() returns new
    values for this data based on pod's current state.
"""

from time import clock

def sim(PodStatus, cmd_int, cmd_ext, *args):
    if PodStatus.state == 1:
        #Do S2A stuff
        pass
    elif PodStatus.state == 3:
        # Do Launching stuff
        pass
    elif PodStatus.state == 5:
        # Do Brake1 stuff
        pass
    elif PodStatus.state == 6:
        # Do Crawling stuff
        pass
    else:
        # Do Brake2 stuff
        pass

    return(PodStatus)


