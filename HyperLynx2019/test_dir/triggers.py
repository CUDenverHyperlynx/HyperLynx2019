def pod_safe(object, *args, **kwargs):
    if object.HV == False and \
        object.speed < 1:
        return(True)
    else:
        return(False)

