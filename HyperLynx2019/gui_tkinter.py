import tkinter as tk
from tkinter import ttk
import socket

class App(tk.Frame):
    def __init__(self, master = None):
        super().__init__(master)

        # set up connection
        HOST = '127.0.0.1'  # Change IP Address when using radios
        PORT = 1028
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((HOST, PORT))

        self.master = master
        self.fontsize=12
        self.create_tabs()
        self.tab1_createWidgets()
        self.tab2_createWidgets()
        self.update()

    def create_tabs(self):
        self.tab_control = ttk.Notebook(m)
        self.tab1 = ttk.Frame(self.tab_control)
        self.tab2 = ttk.Frame(self.tab_control)
        self.tab_control.add(self.tab1, text='Main')
        self.tab_control.add(self.tab2, text='Edit Parameters')
        self.tab_control.pack(expand=1, fill='both')

    def tab1_createWidgets(self):

        # LEFT COLUMN
        self.label_state = tk.Label(self.tab1, text = 'state', font=('verdana',self.fontsize), width=10)
        self.label_state.grid(row=0, column=0)
        self.label_state_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='N/A'), font=('verdana',self.fontsize),
                                        width=20)
        self.label_state_var.grid(row=0, column=1, columnspan=2)

        self.label_true_val = tk.Label(self.tab1, text = 'true_val', font=('verdana',self.fontsize), relief='ridge',
                                       width=40)
        self.label_true_val.grid(row=1, column=0, columnspan=3)
        self.label_true_val_D = tk.Label(self.tab1, text='D', font=('verdana',self.fontsize), width=10)
        self.label_true_val_D.grid(row=2, column=0, columnspan=1)
        self.label_true_val_D_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='N/A'), font=('verdana',self.fontsize),
                                             width=20)
        self.label_true_val_D_var.grid(row=2, column=1, columnspan=1)

        self.label_true_val_V = tk.Label(self.tab1, text='V', font=('verdana', self.fontsize), width=10)
        self.label_true_val_V.grid(row=3, column=0, columnspan=1)
        self.label_true_val_V_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'), font=('verdana', self.fontsize),
                                             width=20)
        self.label_true_val_V_var.grid(row=3, column=1, columnspan=1)

        self.label_true_val_A = tk.Label(self.tab1, text='A', font=('verdana', self.fontsize), width=10)
        self.label_true_val_A.grid(row=4, column=0, columnspan=1)
        self.label_true_val_A_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'), font=('verdana', self.fontsize),
                                             width=20)
        self.label_true_val_A_var.grid(row=4, column=1, columnspan=1)

        self.label_true_val = tk.Label(self.tab1, text = 'Raw Data', font=('verdana',self.fontsize), relief='ridge',
                                       width=40)
        self.label_true_val.grid(row=5, column=0, columnspan=3)

        self.label_throttle = tk.Label(self.tab1, text='throttle', font=('verdana', self.fontsize-2), width=10)
        self.label_throttle.grid(row=6, column=0, columnspan=1)
        self.label_throttle_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'), font=('verdana', self.fontsize-2),
                                             width=20)
        self.label_throttle_var.grid(row=6, column=1, columnspan=1)

        self.label_MET = tk.Label(self.tab1, text='MET', font=('verdana', self.fontsize-2), width=10)
        self.label_MET.grid(row=7, column=0, columnspan=1)
        self.label_MET_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'), font=('verdana', self.fontsize-2),
                                             width=20)
        self.label_MET_var.grid(row=7, column=1, columnspan=1)

        self.label_LIDAR = tk.Label(self.tab1, text='LIDAR', font=('verdana', self.fontsize-2), width=10)
        self.label_LIDAR.grid(row=8, column=0, columnspan=1)
        self.label_LIDAR_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                           font=('verdana', self.fontsize-2),
                                           width=20)
        self.label_LIDAR_var.grid(row=8, column=1, columnspan=1)

        self.label_LST = tk.Label(self.tab1, text='LST Left/Right', font=('verdana', self.fontsize - 2), width=16)
        self.label_LST.grid(row=9, column=0, columnspan=1)
        self.label_LST1_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                        font=('verdana', self.fontsize - 2),
                                        width=18)
        self.label_LST1_var.grid(row=9, column=1, columnspan=1)
        self.label_LST2_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                       font=('verdana', self.fontsize - 2),
                                       width=18)
        self.label_LST2_var.grid(row=9, column=2, columnspan=1)

        self.label_resolver = tk.Label(self.tab1, text='Motor Speed', font=('verdana', self.fontsize-2), width=10)
        self.label_resolver.grid(row=10, column=0, columnspan=1)
        self.label_resolver_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                           font=('verdana', self.fontsize-2),
                                           width=20)
        self.label_resolver_var.grid(row=10, column=1, columnspan=1)

        self.label_imu = tk.Label(self.tab1, text='IMU-Z, [g]', font=('verdana', self.fontsize - 2), width=16)
        self.label_imu.grid(row=11, column=0, columnspan=1)
        self.label_imu1_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                           font=('verdana', self.fontsize - 2),
                                           width=18)
        self.label_imu1_var.grid(row=11, column=1, columnspan=1)
        self.label_imu2_var = tk.Label(self.tab1, textvariable=tk.StringVar(value='N/A'),
                                           font=('verdana', self.fontsize - 2),
                                           width=18)
        self.label_imu2_var.grid(row=11, column=2, columnspan=1)



        # RIGHT COLUMN

        self.button_Launch = tk.Button(self.tab1, text='LAUNCH', width=20, height=1, state='disabled',
                                       font=('verdana', self.fontsize))
        self.button_Launch.grid(row=0, column=5, columnspan=2)
        self.button_Abort = tk.Button(self.tab1, text='ABORT', width=20, height=1, state='disabled', font=('verdana', self.fontsize))
        self.button_Abort.grid(row=0, column=7, columnspan=2)

        self.button_Fault = tk.Button(self.tab1, text='FAULT', width=20, state='disabled', font=('verdana', self.fontsize))
        self.button_Fault.grid(row=1, column=5, columnspan=2)
        self.button_Ready = tk.Button(self.tab1, text='Ready to Launch', width=20, state='disabled', font=('verdana', self.fontsize))
        self.button_Ready.grid(row=1, column=7, columnspan=2)

        self.label_Controls = tk.Label(self.tab1, text='CONTROLS', font=('verdana',self.fontsize), relief='ridge', width=40)
        self.label_Controls.grid(row=2, column=5, columnspan=4)
        self.label_Status = tk.Label(self.tab1, text='STATUS', font=('verdana', self.fontsize), relief='ridge', width=10)
        self.label_Status.grid(row=2, column=9)

        self.button_vent_close = tk.Button(self.tab1, text='Close Vent Sol', width=20, state='disabled', font=('verdana', self.fontsize))
        self.button_vent_close.grid(row=3, column=5, columnspan=2)
        self.button_vent_open = tk.Button(self.tab1, text='Open Vent Sol', width=20, state='disabled',
                                      font=('verdana', self.fontsize))
        self.button_vent_open.grid(row=3, column=7, columnspan=2)
        self.label_vent_status_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='Vent N/A'), width=10, font=('verdana', self.fontsize))
        self.label_vent_status_var.grid(row=3, column=9)

        self.button_res1_close = tk.Button(self.tab1, text='Close Res1', width=20, state='disabled',
                                           font=('verdana', self.fontsize))
        self.button_res1_close.grid(row=4, column=5, columnspan=2)
        self.button_res1_open = tk.Button(self.tab1, text='Open Res1', width=20, state='disabled',
                                          font=('verdana', self.fontsize))
        self.button_res1_open.grid(row=4, column=7, columnspan=2)
        self.label_res1_status_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='Res1 N/A'), width=10, font=('verdana', self.fontsize))
        self.label_res1_status_var.grid(row=4, column=9)

        self.button_res2_close = tk.Button(self.tab1, text='Close Res2', width=20, state='disabled',
                                           font=('verdana', self.fontsize))
        self.button_res2_close.grid(row=5, column=5, columnspan=2)
        self.button_res2_open = tk.Button(self.tab1, text='Open Res2', width=20, state='disabled',
                                          font=('verdana', self.fontsize))
        self.button_res2_open.grid(row=5, column=7, columnspan=2)
        self.label_res2_status_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='Res2 N/A'), width=10, font=('verdana', self.fontsize))
        self.label_res2_status_var.grid(row=5, column=9)


        self.button_hv_pos_on = tk.Button(self.tab1, text='HV(+) ON', width=20, state='disabled',
                                           font=('verdana', self.fontsize))
        self.button_hv_pos_on.grid(row=6, column=5, columnspan=2)
        self.button_hv_pos_off = tk.Button(self.tab1, text='HV(+) OFF', width=20, state='disabled',
                                          font=('verdana', self.fontsize))
        self.button_hv_pos_off.grid(row=6, column=7, columnspan=2)
        self.label_hvp_status_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='HV(+) N/A'), width=10, font=('verdana', self.fontsize))
        self.label_hvp_status_var.grid(row=6, column=9)

        self.button_hv_neg_on = tk.Button(self.tab1, text='HV(-) ON', width=20, state='disabled',
                                           font=('verdana', self.fontsize))
        self.button_hv_neg_on.grid(row=7, column=5, columnspan=2)
        self.button_hv_neg_off = tk.Button(self.tab1, text='HV(-) OFF', width=20, state='disabled',
                                          font=('verdana', self.fontsize))
        self.button_hv_neg_off.grid(row=7, column=7, columnspan=2)
        self.label_hvn_status_var = tk.Label(self.tab1, textvariable = tk.StringVar(value='HV(-) N/A'), width=10, font=('verdana', self.fontsize))
        self.label_hvn_status_var.grid(row=7, column=9)

        # self.label_cmd_state = tk.Label(self.tab1, text='Command State:')
        # self.entry_cmd_state_var = 'idle'
        # self.entry_cmd_state = tk.Entry(self.tab1)
        # self.label_cmd_state.grid(row=1, column=0)
        # self.entry_cmd_state.grid(row=1, column=1)
        # self.button_cmd_state = tk.Button(self.tab1, text='SEND', width=10, command=self.cmd_state)
        # self.button_cmd_state.grid(row=1, column=2)

        # self.button_Abort = tk.Button(self.tab1, text='ABORT', width=10, command=self.abort)
        # self.button_Abort.grid(row=10, column=0)

    def tab2_createWidgets(self):
        self.label_charge = tk.Label(self.tab2, text='Flight parameters page')
        self.label_charge.grid(row=0, column=0)
        self.label_charge_val = tk.Label(self.tab2, textvariable = tk.StringVar(value=None))
        self.label_charge_val.grid(row=0, column=1)

    # def cmd_state(self):
    #     ship.command['state'] = self.entry_cmd_state.get()
    #
    # def abort(self):
    #     ship.v.control.abort = True

    def update(self):
        ### DEBUG ###
        self.state = 'SafeToApproach'
        self.hv = False
        self.vent = 'OPEN'
        self.label_state_var.configure(textvariable = tk.StringVar(value=self.state))
        ### END DEBUG ###

        if self.state == 'SafeToApproach':
            self.label_vent_status_var.config(textvariable=tk.StringVar(value=str(self.vent)))
            self.label_hvn_status_var.config(textvariable=tk.StringVar(value=str(self.hv)))
            self.label_hvp_status_var.config(textvariable=tk.StringVar(value=str(self.hv)))
            if self.vent == 'CLOSED':
                self.button_vent_open.config(state='normal')
                self.button_vent_close.config(state='disabled')
            else:
                self.button_vent_open.config(state='disabled')
                self.button_vent_close.config(state='normal')
            if self.hv:
                self.button_hv_neg_off.config(state='normal', bg='red')
                self.button_hv_neg_on.config(state='disabled', bg='gray')
                self.button_hv_pos_off.config(state='normal', bg='red')
                self.button_hv_pos_on.config(state='disabled', bg='gray')
            else:
                self.button_hv_neg_off.config(state='disabled', bg='gray')
                self.button_hv_neg_on.config(state='normal', bg='red')
                self.button_hv_pos_off.config(state='disabled', bg='gray')
                self.button_hv_pos_on.config(state='normal', bg='red')

        self.after(100, self.update)

if __name__ == '__main__':

    m = tk.Tk()
    m.title('Tkinter GUI')
    m.geometry('1200x800')
    app = App(master=m)
    m.mainloop()
