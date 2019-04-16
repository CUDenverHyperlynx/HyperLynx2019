# Script listens to serial port and writes contents into a file
# requires pySerial to be installed
# sudo pip install pyserial should work
import serial

serial_port = '/dev/cu.usbmodem14101'
# In arduino, Serial.begin(baud_rate)
baud_rate = 9600
write_to_file_path = "output.txt"

output_file = open(write_to_file_path, "w+")
ser = serial.Serial(serial_port, baud_rate)
while True:
    line = ser.readline()
    # ser.readline returns a binary, convert to string
    line = line.decode("utf-8")
    print(line)
    output_file.write(line)
