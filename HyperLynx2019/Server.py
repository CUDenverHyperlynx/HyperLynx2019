
# HyperLynx TCP Server


import socket
import pickle
from queue import Queue
import random
import sys
import os
import yaml
from time import clock
import numpy as np
from PyQt5.QtWidgets import QApplication, QPushButton, QTextEdit, QTableWidget
from PyQt5.QtWidgets import QCheckBox, QSlider, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import *

from PyQt5 import QtCore, Qt
from PyQt5.QtWidgets import QMainWindow, QTextEdit, QTableWidget, QPushButton, QCheckBox, QSlider, QApplication, \
    QTableWidgetItem
from gui_data_simulator import load_abort_ranges
from network_transfer.libserver import BaseServer, ThreadedServer
# from SDA import Status

# set up connection
HOST = '127.0.0.1'  # Change IP Address when using radios
PORT = 1028
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))

# Print Listening on HOST and PORT
print(HOST)
print(PORT)


class HyperGui(QMainWindow):
    # Creating the dictionary
    # cmd_ext = {'abort': 0, 'hv': 0, 'vent_sol': 0, 'res1_sol': 0, 'res2_sol': 0, 'mc_pump': 0}
    ####################################
    # set up connection
    # HOST = '127.0.0.1'  # Change IP Address when using radios
    # PORT = 1028
    # s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # s.bind((HOST, PORT))
    ###################################

    ### Abort range sensors ###
    # 'Ambient_Pressure',
    # 'Brake_Pressure',
    # 'D_diff',
    # 'IMU_bad_time_elapsed',
    # 'LIDAR',
    # 'LVBatt_Current',
    # 'LVBatt_Temp',
    # 'LVBatt_Voltage',
    # 'PV_Left_Pressure',
    # 'PV_Left_Temp',
    # 'PV_Right_Pressure',
    # 'PV_Right_Temp',
    # 'RPi_Mem_Free',
    # 'RPi_Mem_Load',
    # 'RPi_Mem_Used',
    # 'RPi_Proc_Load',
    # 'RPi_Temp',
    # 'V_bad_time_elapsed'

    # ******* Constructor for the class *******
    def __init__(self, host='localhost', port=5050, **kwargs):
        super().__init__()

        self.data_q = Queue()
        self.host = host
        self.port = port
        self.server = ThreadedServer(q=self.data_q, host=self.host, port=self.port)
        self.server.setDaemon(True)
        self.server.start()

        # static for testing, need to change with data
        self.state = ''

        self.abort_ranges = load_abort_ranges('abortranges.dat')

        with open('sensors_config.yaml') as f:
            tables = yaml.safe_load(f)
        self.pod_dyn_nms = tables['pod_dyn_table']
        self.pod_hlth_nms = tables['pod_health']
        self.env_tbl_nms = tables['environment_table']
        self.data_dict = {k:np.nan for v in tables.values() for k in v}

        self.init_ui()

        # This is creating the thread with an interval that calls the function update_txt
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_txt)

    # ******* This class will only initialize the gui *******
    def init_ui(self):
        # ******* This is the gui status text for the program (Need to change design) *******
        self.gui_status = QTextEdit('<b>Live</b>', self)
        self.gui_status.setReadOnly(True)
        self.gui_status.setAlignment(QtCore.Qt.AlignCenter)
        self.gui_status.setToolTip('This is a <b>Status</b> text')
        self.gui_status.resize(80, 30)
        self.gui_status.move(5, 5)

        # ******* This is the state text which can change *******
        self.state_tbox = QTextEdit(self._state_txt(), self)
        self.state_tbox.setReadOnly(True)
        self.state_tbox.setToolTip('This is a <b>State</b> text')
        self.state_tbox.resize(200, 35)
        self.state_tbox.move(100, 5)

        # ******* This is our pod dynamics table (May change things in the future) *******

        # Adjusting the text box for Pod Dynamics
        self.pod_dyn_txt = QTextEdit('<b>POD DYNAMICS</b> ', self)
        self.pod_dyn_txt.setAlignment(QtCore.Qt.AlignCenter)
        self.pod_dyn_txt.setReadOnly(True)
        self.pod_dyn_txt.resize(425, 30)
        self.pod_dyn_txt.move(5, 390)

        #  Adjusting the table for Pod Dynamics
        self.pod_dyn_table = QTableWidget(self)
        self.pod_dyn_table.setRowCount(len(self.pod_dyn_nms))
        self.pod_dyn_table.setColumnCount(4)
        self.pod_dyn_table.resize(425, 320)
        self.pod_dyn_table.move(5, 420)
        self.pod_dyn_table.resizeRowsToContents()

        # ******* This is the time and Met text which will change values *******
        self.time_txt = QTextEdit('', self)
        self.time_txt.append('<h2>Time:</h2> \n <h2>MET:</h2>')
        self.time_txt.setReadOnly(True)
        self.time_txt.setToolTip('This is a <b>Time</b> text')
        self.time_txt.resize(200, 80)
        self.time_txt.move(320, 5)

        # ******* This is the brake vent close button for the program (Need to change design) *******
        self.bv_close = QPushButton('Brake Vent \n CLOSE', self)
        self.bv_close.setToolTip('This is a <b>Brake Vent Close</b> button')
        self.bv_close.setEnabled(False)
        self.bv_close.resize(120, 50)
        self.bv_close.move(0, 60)
        self.bv_close.clicked.connect(self.bv_close_handler)
        self.bv_close.update()

        # ******* This is the brake vent open button for the program (Need to change design) *******
        self.bv_open = QPushButton('Brake Vent \n OPEN', self)
        self.bv_open.setToolTip('This is a <b>Brake Vent Open</b> button')
        self.bv_open.resize(120, 50)
        self.bv_open.move(120, 60)
        self.bv_open.clicked.connect(self.bv_open_handler)
        self.bv_open.update()

        # ******* This is the res #1 close button for the program (Need to change design) *******
        self.res1_close = QPushButton('Res #1 \n CLOSE', self)
        self.res1_close.setToolTip('This is a <b>Res #1 Close</b> button')
        self.res1_close.setEnabled(False)
        self.res1_close.resize(120, 50)
        self.res1_close.move(0, 110)
        self.res1_close.clicked.connect(self.res1_close_handler)
        self.res1_close.update()

        # ******* This is the res #1 open button for the program (Need to change design) *******
        self.res1_open = QPushButton('Res #1 \n OPEN', self)
        self.res1_open.setToolTip('This is a <b>Res #1 Open</b> button')
        self.res1_open.resize(120, 50)
        self.res1_open.move(120, 110)
        self.res1_open.clicked.connect(self.res1_open_handler)
        self.res1_open.update()

        # ******* This is the res #2 close button for the program (Need to change design) *******
        self.res2_close = QPushButton('Res #2 \n CLOSE', self)
        self.res2_close.setToolTip('This is a <b>Res #2 Close</b> button')
        self.res2_close.setEnabled(False)
        self.res2_close.resize(120, 50)
        self.res2_close.move(0, 160)
        self.res2_close.clicked.connect(self.res2_close_handler)
        self.res2_close.update()

        # ******* This is the res #2 open button for the program (Need to change design) *******
        self.res2_open = QPushButton('Res #2 \n OPEN', self)
        self.res2_open.setToolTip('This is a <b>Res #2 Open</b> button')
        self.res2_open.resize(120, 50)
        self.res2_open.move(120, 160)
        self.res2_open.clicked.connect(self.res2_open_handler)
        self.res2_open.update()

        # ******* This is the hv+ off button for the program (Need to change design) *******
        self.hv_plus_off = QPushButton('HV (+) \n OFF', self)
        self.hv_plus_off.setToolTip('This is a <b>HV plus OFF</b> button')
        self.hv_plus_off.setEnabled(False)
        self.hv_plus_off.resize(120, 50)
        self.hv_plus_off.move(0, 210)
        self.hv_plus_off.clicked.connect(self.hv_plus_off_handler)
        self.hv_plus_off.update()

        # ******* This is the hv+ on button for the program (Need to change design) *******
        self.hv_plus_on = QPushButton('HV (+) \n ON', self)
        self.hv_plus_on.setToolTip('This is a <b>HV plus ON</b> button')
        self.hv_plus_on.resize(120, 50)
        self.hv_plus_on.move(120, 210)
        self.hv_plus_on.clicked.connect(self.hv_plus_on_handler)
        self.hv_plus_on.update()

        # ******* This is the hv- off button for the program (Need to change design) *******
        self.hv_minus_off = QPushButton('HV (-) \n OFF', self)
        self.hv_minus_off.setToolTip('This is a <b>HV minus OFF</b> button')
        self.hv_minus_off.setEnabled(False)
        self.hv_minus_off.resize(120, 50)
        self.hv_minus_off.move(0, 260)
        self.hv_minus_off.clicked.connect(self.hv_minus_off_handler)
        self.hv_minus_off.update()

        # ******* This is the hv- on button for the program (Need to change design) *******
        self.hv_minus_on = QPushButton('HV (-) \n ON', self)
        self.hv_minus_on.setToolTip('This is a <b>HV minus ON</b> button')
        self.hv_minus_on.resize(120, 50)
        self.hv_minus_on.move(120, 260)
        self.hv_minus_on.clicked.connect(self.hv_minus_on_handler)
        self.hv_minus_on.update()

        # ******* This is the manual throttle checkbox *******
        self.mt_chkbox = QCheckBox('Manual Throttle', self)
        self.mt_chkbox.setToolTip('This is the <b>Manual Throttle</b> Checkbox')
        self.mt_chkbox.move(35, 330)

        # ******* This is the manual throttle slider *******
        self.mt_chkslider = QSlider(QtCore.Qt.Horizontal, self)
        self.mt_chkslider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.mt_chkslider.setGeometry(35, 350, 110, 30)

        # ******* This is the LAUNCH button for the program (Need to change design) *******
        self.launch_bttn = QPushButton('LAUNCH', self)
        self.launch_bttn.setToolTip('This is the <b>LAUNCH</b> button')
        self.launch_bttn.resize(90, 90)
        self.launch_bttn.move(250, 90)

        # When you click the launch button you start the thread
        self.launch_bttn.clicked.connect(self.start)

        # ******* This is the ABORT button for the program (Need to change design) *******
        self.abort_bttn = QPushButton('ABORT', self)
        self.abort_bttn.setToolTip('This is the <b>ABORT</b> button')
        self.abort_bttn.resize(90, 90)
        self.abort_bttn.move(250, 180)

        # When you click abort button you stop the thread
        self.abort_bttn.clicked.connect(self.stop)

        # ******* This is the log text box *******

        # Creating the Log Text box
        self.pd_log_txt = QTextEdit('Host:' + self.host + ' Port:' + str(self.port), self)
        self.pd_log_txt.setAlignment(QtCore.Qt.AlignCenter)
        self.pd_log_txt.setReadOnly(True)
        self.pd_log_txt.resize(350, 230)
        self.pd_log_txt.move(550, 5)

        # ******* This is the pod health table *******
        hlth_width = 500
        hlth_pos = (950, 5)
        # Creating the Pod Health text
        self.pod_hlth_txt = QTextEdit('<b>Pod Health</b>', self)
        self.pod_hlth_txt.setAlignment(QtCore.Qt.AlignCenter)
        self.pod_hlth_txt.setReadOnly(True)
        self.pod_hlth_txt.resize(hlth_width, 30)
        self.pod_hlth_txt.move(*hlth_pos)

        # Creating the table for Pod Health
        self.pod_hlth_table = QTableWidget(self)
        self.pod_hlth_table.setRowCount(len(self.pod_hlth_nms))
        self.pod_hlth_table.setColumnCount(3)
        self.pod_hlth_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        self.pod_hlth_table.setVerticalHeaderLabels([k for k in self.pod_hlth_nms.values()])
        self.pod_hlth_table.resize(hlth_width, 380)
        self.pod_hlth_table.move(hlth_pos[0], hlth_pos[1]+30)
        self.pod_hlth_table.resizeRowsToContents()

        # ******* This is the Environmentals table *******
        env_width = 500
        env_pos = (950, 420)
        # Creating the Environmentals text
        self.env_txt = QTextEdit('<b>Environmentals</b>', self)
        self.env_txt.setAlignment(QtCore.Qt.AlignCenter)
        self.env_txt.setReadOnly(True)
        self.env_txt.resize(env_width, 30)
        self.env_txt.move(*env_pos)

        # Creating the table for Environmentals
        self.env_table = QTableWidget(self)
        self.env_table.setRowCount(len(self.env_tbl_nms))
        self.env_table.setColumnCount(3)
        self.env_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        self.env_table.setVerticalHeaderLabels([k for k in self.env_tbl_nms.values()])
        self.env_table.resize(env_width, 295)
        self.env_table.move(env_pos[0], env_pos[1]+30)

        # setGeometry has 4 values to pass in the first two are window position in relation to your computer (x, y)
        # the second two values are the size of the window itself (width, height)
        self.setGeometry(0, 0, 2000, 1000)

        # Sets the title of the window
        self.setWindowTitle('HyperLynx GUI')

    # ******* Other Functions for the class *******

    # This function will start the thread
    def start(self):
        self.state = 1
        self.timer.start()

    # This function will stop the thread
    def stop(self):
        self.timer.stop()

    # This function will handle when bv close button is clicked
    def bv_close_handler(self):
        self.bv_close.setEnabled(False)
        self.bv_open.setEnabled(True)
        self.update()

    # This function will handle when bv open button is clicked
    def bv_open_handler(self):
        self.bv_close.setEnabled(True)
        self.bv_open.setEnabled(False)
        self.update()

    # This function will handle when res1 close button is clicked
    def res1_close_handler(self):
        self.res1_close.setEnabled(False)
        self.res1_open.setEnabled(True)
        self.update()

    # This function will handle when res1 open button is clicked
    def res1_open_handler(self):
        self.res1_close.setEnabled(True)
        self.res1_open.setEnabled(False)
        self.update()

    # This function will handle when res2 close button is clicked
    def res2_close_handler(self):
        self.res2_close.setEnabled(False)
        self.res2_open.setEnabled(True)
        self.update()

    # This function will handle when res2 open button is clicked
    def res2_open_handler(self):
        self.res2_close.setEnabled(True)
        self.res2_open.setEnabled(False)
        self.update()

    # This function will handle when hv+ off close button is clicked
    def hv_plus_off_handler(self):
        self.hv_plus_off.setEnabled(False)
        self.hv_plus_on.setEnabled(True)
        self.update()

    # This function will handle when hv+ on button is clicked
    def hv_plus_on_handler(self):
        self.hv_plus_off.setEnabled(True)
        self.hv_plus_on.setEnabled(False)
        self.update()

    # This function will handle when hv- off button is clicked
    def hv_minus_off_handler(self):
        self.hv_minus_off.setEnabled(False)
        self.hv_minus_on.setEnabled(True)
        self.update()

    # This function will handle when hv- on button is clicked
    def hv_minus_on_handler(self):
        self.hv_minus_off.setEnabled(True)
        self.hv_minus_on.setEnabled(False)
        self.update()

    def _state_txt(self, state=None):
        if not state:
            return '<h2>State:</h2> '
        else:
            return '<h2>State: {}</h2> '.format(state)
    
    
    # def _read_status(self, pStatus):
    #     # dynamics
    #     self.data_dict['pos'] = pStatus.true_data['D']['val']
    #     self.data_dict['stp_cnt'] = pStatus.true_data['stripe_count']
    #     self.data_dict['spd'] = pStatus.true_data['V']['val']
    #     self.data_dict['accl'] = pStatus.true_data['A']['val']
    #     self.data_dict['IMU1_Z'] = pStatus.sensor_filter['IMU1_Z']['val']
    #     self.data_dict['IMU2_z'] = pStatus.sensor_filter['IMU2_Z']['val']
    #     self.data_dict['thrtl'] = pStatus.throttle
    #     self.data_dict['lidar'] = pStatus.sensor_filter['LIDAR']['val']


    # This function is running on the thread separate from initializing the gui
    def update_txt(self):
        # Random number generator
        test_val = random.uniform(0, 3)

        # check for new data
        if not self.data_q.empty():
            self.data_dict.update(self.data_q.get())
        #     pstatus = pickle.loads(self.data_q.get())
        #     self._read_status(pstatus)

        # update current state
        state = self.state
        self.state_tbox.setText(self._state_txt(state))

        # Update tables
        # Update Dynamics Table
        for i,k in enumerate(self.pod_dyn_nms):
            value = self.data_dict.get(k, 'err')
            if type(value) in {int, float}:
                self.pod_dyn_table.setItem(i, 1, QTableWidgetItem(
                    '{:.2f}'.format(value)
                ))
            else:
                self.pod_dyn_table.setItem(i, 1, QTableWidgetItem(
                    '{}'.format(value)
                ))
        # Upate Health table
        for i,k in enumerate(self.pod_hlth_nms):
            sensor = self.abort_ranges[state].get(k, {'Low':'', 'High':''})
            self.pod_hlth_table.setItem(i, 0, QTableWidgetItem(
                '{}'.format(sensor['Low'])
            ))
            value = self.data_dict.get(k, 'err')
            if type(value) in {int, float}:
                self.pod_hlth_table.setItem(i, 1, QTableWidgetItem(
                    '{:.2f}'.format(value)
                ))
            else:
                self.pod_hlth_table.setItem(i, 1, QTableWidgetItem(
                    '{}'.format(value)
                ))
            self.pod_hlth_table.setItem(i, 2, QTableWidgetItem(
                '{}'.format(sensor['High'])
            ))

        # Update Environment table
        for i,k in enumerate(self.env_tbl_nms):
            sensor = self.abort_ranges[state].get(k, {'Low':'', 'High':''})
            self.env_table.setItem(i, 0, QTableWidgetItem('{}'.format(
                sensor['Low']
            )))
            value = self.data_dict.get(k, 'err')
            if type(value) in {int, float}:
                self.env_table.setItem(i, 1, QTableWidgetItem(
                    '{:.2f}'.format(value)
                ))
            else:
                self.env_table.setItem(i, 1, QTableWidgetItem(
                    '{}'.format(value)
                ))
            self.env_table.setItem(i, 2, QTableWidgetItem('{}'.format(
                sensor['High']
            )))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='GUI for Hyperlynx 2019 pod')
    parser.add_argument('--server', help='<host>:<port>')
    args = parser.parse_args()

    params = {
        'host': 'localhost',
        'port': 5050,
        'abort_ranges': None
    }
    if args.server:
        host, port = args.server.split(':')
        params.update({'host': host, 'port': int(port)})

    app = QApplication([])
    my_gui = HyperGui(**params)
    my_gui.show()

    sys.exit(app.exec_())

'''
# loop during run
while 1:
    # Receive new data
    data, address = s.recvfrom(50000)
    data_variable = pickle.loads(data)

    # Print all data that was in the log file to the console
    for key in data_variable.sensor_data:
        if str(key) in data_variable.abort_ranges[data_variable.state]:
            fault_code = data_variable.abort_ranges[data_variable.state][str(key)]['Fault']
        else:
            fault_code = 0

        print(str(key) + '\t' + str(data_variable.sensor_data[str(key)]) + '\t' +
              str(int(fault_code)) + '\t' + str(round(clock(), 2)))

    for key in data_variable.commands:
        print(str(key) + '\t' + str(data_variable.commands[str(key)]) + '\t\t' + str(round(clock(), 2)))

    print('state' + '\t' + str(data_variable.state) + '\t\t' + str(round(clock(), 2)) + '\n' + 'spacex_state' + '\t'
          + str(data_variable.spacex_state) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'total_faults' + '\t' + str(data_variable.total_faults) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'throttle' + '\t' + str(data_variable.throttle) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'distance' + '\t' + str(data_variable.distance) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'speed' + '\t' + str(data_variable.speed) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'accel' + '\t' + str(data_variable.accel) + '\t\t' + str(round(clock(), 2)) + '\n'
          + 'wheel_diameter' + '\t' + str(data_variable.wheel_diameter) + '\t\t' + str(round(clock(), 2)) + '\n')

    # Call GUI function and send data
    # Put GUI function here
'''