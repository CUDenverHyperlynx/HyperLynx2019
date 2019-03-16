class State():
    def __init__(self):
        self.state = 0

def init(var):
    var.state = 1

if __name__ == '__main__':
    PodStatus = State()
    init(PodStatus)
    print(PodStatus.state)