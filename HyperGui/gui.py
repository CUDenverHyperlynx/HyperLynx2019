# Made by Mohammad Hossain
# For CUDenver HyperLynx Project 2019

import sys
import random

from PyQt5.QtWidgets import QApplication, QPushButton, QTextEdit, QTableWidget, QCheckBox, QSlider, QMainWindow, \
    QTableWidgetItem
from PyQt5.QtCore import *
from PyQt5 import QtCore


# GUI class
class HyperGui(QMainWindow):
    # Creating the dictionary
    cmd_ext = {'abort': 0, 'hv': 0, 'vent_sol': 0, 'res1_sol': 0, 'res2_sol': 0, 'mc_pump': 0}

    # ******* Constructor for the class *******
    def __init__(self):
        super().__init__()

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
        self.gui_status.setAlignment(Qt.AlignCenter)
        self.gui_status.setToolTip('This is a <b>Status</b> text')
        self.gui_status.resize(80, 30)
        self.gui_status.move(5, 5)

        # ******* This is the state text which can change *******
        self.state_txt = QTextEdit('<h2>State:</h2> ', self)
        self.state_txt.setReadOnly(True)
        self.state_txt.setToolTip('This is a <b>State</b> text')
        self.state_txt.resize(200, 35)
        self.state_txt.move(100, 5)

        # ******* This is our pod dynamics table (May change things in the future) *******

        # Adjusting the text box for Pod Dynamics
        self.pod_dyn_txt = QTextEdit('<b>POD DYNAMICS</b> ', self)
        self.pod_dyn_txt.setAlignment(Qt.AlignCenter)
        self.pod_dyn_txt.setReadOnly(True)
        self.pod_dyn_txt.resize(425, 30)
        self.pod_dyn_txt.move(5, 390)

        #  Adjusting the table for Pod Dynamics
        self.pod_dyn_table = QTableWidget(self)
        self.pod_dyn_table.setRowCount(22)
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
        self.mt_chkslider = QSlider(Qt.Horizontal, self)
        self.mt_chkslider.setFocusPolicy(Qt.NoFocus)
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
        self.pd_val_txt = QTextEdit('<b>Pod Value</b>', self)
        self.pd_val_txt.setAlignment(Qt.AlignCenter)
        self.pd_val_txt.setReadOnly(True)
        self.pd_val_txt.resize(150, 230)
        self.pd_val_txt.move(550, 5)

        # ******* This is the pod health table *******

        # Creating the Pod Health text
        self.pod_hlth_txt = QTextEdit('<b>Pod Health</b>', self)
        self.pod_hlth_txt.setAlignment(Qt.AlignCenter)
        self.pod_hlth_txt.setReadOnly(True)
        self.pod_hlth_txt.resize(325, 30)
        self.pod_hlth_txt.move(950, 5)

        # Creating the table for Pod Health
        self.pod_hlth_table = QTableWidget(self)
        self.pod_hlth_table.setRowCount(24)
        self.pod_hlth_table.setColumnCount(3)
        self.pod_hlth_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        self.pod_hlth_table.resize(325, 380)
        self.pod_hlth_table.move(950, 35)
        self.pod_hlth_table.resizeRowsToContents()

        # ******* This is the Environmentals table *******

        # Creating the Environmentals text
        self.env_txt = QTextEdit('<b>Environmentals</b>', self)
        self.env_txt.setAlignment(Qt.AlignCenter)
        self.env_txt.setReadOnly(True)
        self.env_txt.resize(321, 30)
        self.env_txt.move(950, 420)

        # Creating the table for Environmentals
        self.env_table = QTableWidget(self)
        self.env_table.setRowCount(9)
        self.env_table.setColumnCount(3)
        self.env_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        self.env_table.resize(321, 295)
        self.env_table.move(950, 450)

        # setGeometry has 4 values to pass in the first two are window position in relation to your computer (x, y)
        # the second two values are the size of the window itself (width, height)
        self.setGeometry(0, 0, 2000, 1000)

        # Sets the title of the window
        self.setWindowTitle('HyperLynx GUI')

    # ******* Other Functions for the class *******

    # This function will start the thread
    def start(self):
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

    # This function is running on the thread separate from initializing the gui
    def update_txt(self):
        loop_cnt = 0

        # Its arbitrary but if I did something like while True it breaks the GUI
        while loop_cnt < 50:
            # Random number generator
            test_val = random.uniform(0, 3)

            # Update the items of the environmental table
            self.env_table.setItem(0, 0, QTableWidgetItem("0.0"))
            self.env_table.setItem(0, 1, QTableWidgetItem(str(test_val)))
            self.env_table.setItem(0, 2, QTableWidgetItem("3.0"))

            loop_cnt += 1


# ******* Running the main application *******

app = QApplication([])
my_gui = HyperGui()
my_gui.show()
sys.exit(app.exec_())
