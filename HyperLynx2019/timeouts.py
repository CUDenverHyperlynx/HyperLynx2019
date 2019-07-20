# stores timeouts for each state


def get(self):
    self.timeouts[0] = 60
    self.timeouts[1] = 3600
    self.timeouts[2] = 15
    self.timeouts[3] = 60
    self.timeouts[5] = 30
    self.timeouts[6] = 180
    self.timeouts[7] = 30
    return self.timeouts