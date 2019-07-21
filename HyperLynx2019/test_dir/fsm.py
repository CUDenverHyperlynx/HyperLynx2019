from transitions import Machine
import events, triggers

class FSM(object):
    def __init__(self, name):
        self.name = name

        self.HV = False
        self.speed = 0

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

        transitions = [
            {'source': 'Safing', 'trigger': 'pod_safe', 'dest': 'SafeToApproach', conditions=triggers.pod_safe()},
            {'source': 'SafeToApproach', 'trigger': 'cmd_launch', 'dest': 'Prelaunch'},
            {'source': 'Prelaunch', 'trigger': 'prelaunch_ok', 'dest': 'Launching'},
            {'source': 'Prelaunch', 'trigger': 'cmd_abort', 'dest': 'Safing'}
        ]

        self.machine = Machine(model=self, states=state_list, transitions=transitions,
                               initial='Safing')
        self.telem = {'speed':100}

    def eval_trans(self):
        if triggers.pod_safe(self): self.pod_safe()
        pass

    def auto_events(self):
        eventname = getattr(events, self.state)
        eventname(self)
        self.eval_trans()

if __name__ == '__main__':
    fsm = FSM('Hyperlynx')
    while not fsm.state == 'Launching':
        fsm.auto_events()

    print('Finished in state: {}'.format(fsm.state))
