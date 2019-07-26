# Stock modules
from transitions import Machine
import time

# Custom modules
import events, ext_conditions, console_log

class Telemetry():
    def __init__(self):
        self.HV = True
        self.speed = 1
        self.current_time = time.clock()

class FSM(object):
    def __init__(self, name):
        self.name = name
        self.commands = Commands()
        self.telem = Telemetry()
        state_list = [
            'SafeToApproach',
            'Prelaunch',
            'Launching',
            'Brake',
            'Reconfig 1',
            'Reconfig 2',
            'Reconfig 3',
            'Crawling',
            'Brake Final',
            'Safing'
        ]

        # transitions = [
        #     {'source': 'Safing', 'trigger': 'pod_safe', 'dest': 'SafeToApproach', conditions=triggers.pod_safe()},
        #     {'source': 'SafeToApproach', 'trigger': 'cmd_launch', 'dest': 'Prelaunch'},
        #     {'source': 'Prelaunch', 'trigger': 'prelaunch_ok', 'dest': 'Launching'},
        #     {'source': 'Prelaunch', 'trigger': 'cmd_abort', 'dest': 'Safing'}
        # ]

        # initiate state machine
        self.machine = Machine(model=self, states=state_list, initial='Safing')

        # add transitions to state machine
        self.machine.add_transition(source='Safing', trigger='pod_safe', dest='SafeToApproach')
        self.machine.add_transition(source='SafeToApproach', trigger='cmd_launch', dest='Prelaunch')
        self.machine.add_transition(source='Prelaunch', trigger='prelaunch_ok', dest='Launching')
        self.machine.add_transition(source='Prelaunch', trigger='cmd_abort', dest='Safing')

        self.last_known_state = str()

    def eval_triggers(self):
        # find all triggers in state machine
        for m in self.machine.get_triggers(self.state):
            # if the state machine trigger has an external condition:
            if m in dir(ext_conditions):
                # do external condition check
                ext_condition_func = getattr(ext_conditions, m)
                if ext_condition_func(self.telem, self.commands):
                    # if external condition check passes, do state machine transition
                    state_machine_trans_func = getattr(self, m)
                    state_machine_trans_func()


    def auto_events(self):
        self.last_known_state = self.state
        eventname = getattr(self, 'state')
        func = getattr(events, eventname)
        self.telem = func(self.telem)

class Commands:
    def __init__(self):
        self.launch = False
        self.abort = False

if __name__ == '__main__':
    fsm = FSM('Hyperlynx')
    while not fsm.state == 'Launching' and \
            (time.clock()-fsm.telem.current_time) < 5:  # 5 second timeout
        if fsm.last_known_state != fsm.state:
            fsm.auto_events()
            fsm.eval_triggers()
        else:
            print('No change in state: {}'.format(fsm.state))
            time.sleep(0.1)

    print('Finished in state: {}'.format(fsm.state))
