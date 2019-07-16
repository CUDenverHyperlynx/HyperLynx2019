import socket
import json

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('192.168.0.6', 5050)

col_to_state = {'1 - S2A': "SafeToApproach",
           '2 - FC2l': "",
           '3 - Launch': 'Launching',
           '5 - Brake1': 'BrakingHigh',
           'Postflight': "",
           '6 - Crawling': 'Crawling',
           '7 - Brake2': 'BrakingLow',
           }

message = json.dumps(col_to_state).encode('utf-8')

try:

    # Send data
    print('Sending "%s"' % message)
    sent = sock.sendto(message, server_address)
    print()

    # Receive response
    print('Waiting to receive')
    data, server = sock.recvfrom(50000)
    print('Received "%s"' % data)
    print()

finally:
    print('Closing socket')
    sock.close()
