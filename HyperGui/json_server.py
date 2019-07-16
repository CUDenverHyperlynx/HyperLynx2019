import socket
import json

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('192.168.0.6', 5050)
print('Starting up on %s port %s' % server_address)
sock.bind(server_address)

data = b''

while True:
    print()
    print('Waiting to receive message')
    data, address = sock.recvfrom(50000)

    d = json.loads(data.decode('utf-8'))
    print('Received %s bytes from %s' % (len(d), address))
    print(d)

    if d:
        d = json.dumps(d).encode('utf-8')
        sent = sock.sendto(d, address)
        print('Sent %s bytes back to %s' % (sent, address))
