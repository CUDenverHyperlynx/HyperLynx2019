# Jose Ortega
# HyperLynx UDP Client

# Imports
import socket
import pickle
import Packet_Structure

# def send_server(pod_status):    # This function will receive a class and send data over to server.py
# Setting up where to send the data
host = '10.136.167.87'                 # Change IP Address to server IP when running with radios
port = 1028
server_address = (host, port)

# Create a socket connection.
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pod_status = Packet_Structure.Status()

# Pickle the object and send it to the server
data_string = pickle.dumps(pod_status)
s.sendto(data_string, server_address)

print('Data sent to server')

# close connection
s.close()


'''
if __name__ == "__main__":

    # Create Variable from Packet_Structure in order to send data
    send_data = Packet_Structure.Status()

    # Copy all existing data in to new structure
    send_data.abort_ranges = pod_status.abort_ranges
    send_data.commands = pod_status.commands
    send_data.sensor_data = pod_status.sensor_data # contains RPM,
    send_data.sensor_filter = pod_status.sensor_filter
    send_data.true_data = pod_status.true_data
    send_data.poll_oldtime = pod_status.poll_oldtime
    send_data.poll_newtime = pod_status.poll_newtime
'''