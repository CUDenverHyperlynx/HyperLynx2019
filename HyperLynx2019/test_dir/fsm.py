from transitions import Machine
import events

class FSM(object):
    def __init__(self, name):
        self.name = name

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
            {'source': 'Safing', 'trigger': 'pod_safe', 'dest': 'SafeToApproach'},
            {'source': 'SafeToApproach', 'trigger': 'cmd_launch', 'dest': 'Prelaunch'},
            {'source': 'Prelaunch', 'trigger': 'prelaunch_ok', 'dest': 'Launching'},
            {'source': 'Prelaunch', 'trigger': 'cmd_abort', 'dest': 'Safing'}
        ]

        self.machine = Machine(model=self, states=state_list, transitions=transitions,
                               initial='Safing')
        self.telem = {'speed':100}

    def state_event(self):
        if self.state == 'Safing': events.Safing(self)
        elif self.state == 'SafeToApproach': events.SafeToApproach(self)
        elif self.state == 'Prelaunch': events.Prelaunch(self)
        elif self.state == 'Launching': events.Launching(self, self.telem)
        else: print('Error state')

if __name__ == '__main__':
    fsm = FSM('Hyperlynx')
    while not fsm.state == 'Launching':
        fsm.state_event()

    print('Finished in state: {}'.format(fsm.state))
