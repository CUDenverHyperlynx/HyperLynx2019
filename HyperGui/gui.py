# Made by Mohammad Hossain
# For CUDenver HyperLynx Project 2019

import sys
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget, QTextEdit, QTableWidget, QCheckBox, QSlider
from PyQt5.QtCore import *


# ******* This is the GUI Class *******

class HyperGui(QWidget):

    # ******* This is to initialize the class *******
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):

        # ******* This is the status button for the program (Need to change design) *******
        status_btn = QPushButton('Live', self)
        status_btn.setToolTip('This is a <b>Status</b> button')
        status_btn.move(0, 0)

        # ******* This is the status text which can change *******
        status_txt = QTextEdit('<h2>Status:</h2> ', self)
        status_txt.setReadOnly(True)
        status_txt.setToolTip('This is a <b>Status</b> text')
        status_txt.resize(200, 35)
        status_txt.move(80, 5)

        # ******* This is our pod dynamics table (May change things in the future) *******

        # Creating the text box for Pod Dynamics
        pod_dyn_txt = QTextEdit('<b>POD DYNAMICS</b> ', self)
        pod_dyn_txt.setAlignment(Qt.AlignCenter)
        pod_dyn_txt.setReadOnly(True)
        pod_dyn_txt.resize(425, 30)
        pod_dyn_txt.move(0, 390)

        #  Creating the table for Pod Dynamics
        pod_dyn_table = QTableWidget(self)
        pod_dyn_table.setRowCount(22)
        pod_dyn_table.setColumnCount(4)
        pod_dyn_table.resize(425, 300)
        pod_dyn_table.move(0, 420)

        # ******* This is the time and Met text which will change values *******
        time_txt = QTextEdit('', self)
        time_txt.append('<h2>Time:</h2> \n <h2>MET:</h2>')
        time_txt.setReadOnly(True)
        time_txt.setToolTip('This is a <b>Time</b> text')
        time_txt.resize(200, 80)
        time_txt.move(300, 5)

        # ******* This is the brake vent close button for the program (Need to change design) *******
        bv_close = QPushButton('Brake Vent \n CLOSE', self)
        bv_close.setToolTip('This is a <b>Brake Vent Close</b> button')
        bv_close.move(0, 60)

        # ******* This is the brake vent open button for the program (Need to change design) *******
        bv_open = QPushButton('Brake Vent \n OPEN', self)
        bv_open.setToolTip('This is a <b>Brake Vent Open</b> button')
        bv_open.move(120, 60)

        # ******* This is the res #1 close button for the program (Need to change design) *******
        res1_close = QPushButton('Res #1 \n CLOSE', self)
        res1_close.setToolTip('This is a <b>Res #1 Close</b> button')
        res1_close.move(0, 110)

        # ******* This is the res #1 open button for the program (Need to change design) *******
        res1_open = QPushButton('Res #1 \n OPEN', self)
        res1_open.setToolTip('This is a <b>Res #1 Open</b> button')
        res1_open.move(120, 110)

        # ******* This is the res #2 close button for the program (Need to change design) *******
        res2_close = QPushButton('Res #2 \n CLOSE', self)
        res2_close.setToolTip('This is a <b>Res #2 Close</b> button')
        res2_close.move(0, 160)

        # ******* This is the res #2 open button for the program (Need to change design) *******
        res2_open = QPushButton('Res #2 \n OPEN', self)
        res2_open.setToolTip('This is a <b>Res #2 Open</b> button')
        res2_open.move(120, 160)

        # ******* This is the hv+ close button for the program (Need to change design) *******
        hv_plus_close = QPushButton('HV (+) \n CLOSE', self)
        hv_plus_close.setToolTip('This is a <b>HV plus Close</b> button')
        hv_plus_close.move(0, 210)

        # ******* This is the hv+ open button for the program (Need to change design) *******
        hv_plus_open = QPushButton('HV (+) \n OPEN', self)
        hv_plus_open.setToolTip('This is a <b>HV plus Open</b> button')
        hv_plus_open.move(120, 210)

        # ******* This is the hv- close button for the program (Need to change design) *******
        hv_minus_close = QPushButton('HV (-) \n CLOSE', self)
        hv_minus_close.setToolTip('This is a <b>HV minus Close</b> button')
        hv_minus_close.move(0, 260)

        # ******* This is the hv- open button for the program (Need to change design) *******
        hv_minus_open = QPushButton('HV (-) \n OPEN', self)
        hv_minus_open.setToolTip('This is a <b>HV minus Open</b> button')
        hv_minus_open.move(120, 260)

        # ******* This is the manual throttle checkbox *******
        mt_chkbox = QCheckBox('Manual Throttle', self)
        mt_chkbox.setToolTip('This is the <b>Manual Throttle</b> Checkbox')
        mt_chkbox.move(35, 330)

        # ******* This is the manual throttle slider *******
        mt_chkslider = QSlider(Qt.Horizontal, self)
        mt_chkslider.setFocusPolicy(Qt.NoFocus)
        mt_chkslider.setGeometry(35, 350, 110, 30)

        # ******* This is the LAUNCH button for the program (Need to change design) *******
        launch_bttn = QPushButton('LAUNCH', self)
        launch_bttn.setToolTip('This is a <b>LAUNCH</b> button')
        launch_bttn.resize(100, 100)
        launch_bttn.move(250, 120)

        # ******* This is the ABORT button for the program (Need to change design) *******
        abort_bttn = QPushButton('ABORT', self)
        abort_bttn.setToolTip('This is a <b>ABORT</b> button')
        abort_bttn.resize(100, 100)
        abort_bttn.move(250, 230)

        # ******* This is the pod health table *******

        # Creating the Pod Health text
        pod_hlth_txt = QTextEdit('<b>Pod Health</b>', self)
        pod_hlth_txt.setAlignment(Qt.AlignCenter)
        pod_hlth_txt.setReadOnly(True)
        pod_hlth_txt.resize(325, 30)
        pod_hlth_txt.move(750, 5)

        # Creating the table for Pod Health
        pod_hlth_table = QTableWidget(self)
        pod_hlth_table.setRowCount(24)
        pod_hlth_table.setColumnCount(3)
        pod_hlth_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        pod_hlth_table.resize(325, 300)
        pod_hlth_table.move(750, 35)

        # ******* This is the Environmentals table *******

        # Creating the Environmentals text
        env_txt = QTextEdit('<b>Environmentals</b>', self)
        env_txt.setAlignment(Qt.AlignCenter)
        env_txt.setReadOnly(True)
        env_txt.resize(321, 30)
        env_txt.move(750, 390)

        # Creating the table for Environmentals
        env_table = QTableWidget(self)
        env_table.setRowCount(9)
        env_table.setColumnCount(3)
        env_table.setHorizontalHeaderLabels(["LOW", "ACTUAL", "HIGH"])
        env_table.resize(321, 295)
        env_table.move(750, 420)

        # setGeometry has 4 values to pass in the first two are window position in relation to your computer (x, y)
        # the second two values are the size of the window itself (width, height)
        self.setGeometry(40, 50, 1200, 730)

        # Sets the title of the window
        self.setWindowTitle('HyperLynx GUI')
        # .show() basically allows the ui to be shown in the window
        self.show()


# ******* Running the main application *******

app = QApplication([sys.argv])
my_gui = HyperGui()
app.exec_()
