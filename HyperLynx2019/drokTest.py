import RPi.GPIO as IO
from time import sleep

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(17, IO.OUT, initial=IO.LOW)

if __name__ == '__main__':
    while True:
        IO.output(17, IO.HIGH)
        sleep(0.5)
        IO.output(17, IO.LOW)
        sleep(0.5)
