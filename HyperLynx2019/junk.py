import numpy

class State():
    abort_ranges_S2A = {}
    abort_ranges_Launching = {}
    abort_ranges_Brake1 = {}
    abort_ranges_Crawling = {}

    abort_names = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(0, 1), dtype=str)
    abort_vals = numpy.genfromtxt('abortranges.dat', skip_header=1, delimiter='\t', usecols=numpy.arange(1,12), dtype=int)
    for i in range(0,len(abort_names)):
        if abort_vals[i,2] == 1:
            abort_ranges_S2A[abort_names[i]] = {'Low': { abort_vals[i,0]},
                                 'High': { abort_vals[i,1]},
                                 'Trigger': { abort_vals[i,9] },
                                 'Fault': {abort_vals[i, 10]}
                                 }
        if abort_vals[i,4] == 1:
            abort_ranges_Launching[abort_names[i]] = {'Low': { abort_vals[i,0]},
                                 'High': { abort_vals[i,1]},
                                 'Trigger': { abort_vals[i,9] },
                                 'Fault': {abort_vals[i, 10]}
                                 }
        if abort_vals[i,5] == 1:
            abort_ranges_Brake1[abort_names[i]] = {'Low': { abort_vals[i,0]},
                                 'High': { abort_vals[i,1]},
                                 'Trigger': { abort_vals[i,9] },
                                 'Fault': {abort_vals[i, 10]}
                                 }
        if abort_vals[i,6] == 1:
            abort_ranges_Crawling[abort_names[i]] = {'Low': { abort_vals[i,0]},
                                 'High': { abort_vals[i,1]},
                                 'Trigger': { abort_vals[i,9] },
                                 'Fault': {abort_vals[i, 10]}
                                 }
PodStatus = State()
print(PodStatus.abort_ranges_Launching)

