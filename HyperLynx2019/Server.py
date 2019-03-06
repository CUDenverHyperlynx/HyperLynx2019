# Jose Ortega
# HyperLynx TCP Server

import socket
import pickle
import time
import sys

# open file to log data
file = open('testfile.txt', 'w')

# used for run time of server
start_time = time.time()

# set up connection
HOST = socket.gethostbyname(socket.gethostname())
PORT = 1028
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

s.listen(1)

# print out Host and port it's listening on
print('Current time:\t', time.time() - start_time,
      '\nListening on\tHost: ', HOST, '\tPort: ', PORT)

file.write('Current time:\t')
file.write(str(time.time() - start_time))
file.write('\nListening on\nHost: ')
file.write(HOST)
file.write('\tPort: ')
file.write(str(PORT))
file.write('\n\n')

# connect and print what it's connected to
conn, addr = s.accept()
print('\nConnection from client made at:\t', time.time() - start_time)
print('Connected on', addr)

file.write('Connection from client made at:\t')
file.write(str(time.time()-start_time))
file.write('\nConnection on\t')
file.write(str(addr))
file.write('\n\n')

# loop during run
while 1:
    try:
        # receive new data
        data = conn.recv(1028)
        data_variable = pickle.loads(data)

        print('--------------------------')
        print('Data received from client at time\t', time.time()-start_time)
        print('x:\t', data_variable.x)
        print('y:\t', data_variable.y)
        print('--------------------------\n')

        # Access data by data_variable.x, data_variable.y, etc...
        # Write data to file
        file.write('\"Time\" : \"')
        file.write(str(time.time()-start_time))
        file.write('\"\n\"x\" : \"')
        file.write(str(data_variable.x))
        file.write('\"\n\"y\" : \"')
        file.write(str(data_variable.y))
        file.write('\"\n\n')
    except:
        print('Connection has been disconnected')
        file.write('Connection has been disconnected at ')
        file.write(str(time.time() - start_time))
        break

# close file and connection
file.close()
conn.close()
