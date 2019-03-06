# Jose Ortega
# HyperLynx TCP Client

import socket
import pickle
import time
import PacketStructure

# used for run time of server
start_time = time.time()

HOST = '10.134.126.112'
PORT = 1028

# Create a socket connection.
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

for i in range(0, 1001):
    # Create an instance of ProcessData() to send to server.
    variable = PacketStructure.test(time.time() - start_time, i * 2, i * 3)

    # Pickle the object and send it to the server
    data_string = pickle.dumps(variable)
    s.send(data_string)

    # wait 0.1s
    time.sleep(.05)
    print('Data sent to server')

# close connection
s.close()
