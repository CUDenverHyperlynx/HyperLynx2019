
def pod_safe(object, *args):
    if object.HV == False and \
        object.speed < 1:
        print('triggers.pod_safe() is True')
        return(True)
    else:
        print(object.HV+object.speed)
        return(False)

def cmd_launch(object, *args):
    if 'commands' in dir(args):
        if args.launch == True and object.commands.abort == False:
            return(True)
        else:
            return(False)