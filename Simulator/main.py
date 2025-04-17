import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
from tkinter import messagebox
import threading

class Main(tk.Tk):
    def __init__(self):
        super().__init__()
        self.call('source', 'azure.tcl')
        self.call('set_theme', 'light')
        self.title("Төмөр зам симулятор")
        self.configure(padx=(10), pady=(10))
        self.geometry('1000x800')

        self.ser = serial.Serial()
        self.thread = threading.Thread()
        self.stop_event = threading.Event()

        self.port = tk.StringVar()
        self.baudrate = tk.IntVar(value=115200)
        self.con = tk.StringVar(value='Connect')

        self.direction = tk.DoubleVar(value = 0.0)
        self.speed = tk.DoubleVar(value = 0.0)
        self.angle = tk.DoubleVar(value = 0)

        self.m1 = tk.DoubleVar(value=1)
        self.m2 = tk.DoubleVar(value=1)
        self.m3 = tk.DoubleVar(value=1)
        self.m4 = tk.DoubleVar(value=1)
        self.accel = tk.DoubleVar(value = 0.5)
        self.rotate_scale = tk.DoubleVar(value = 0.5)

        self.shot_speed = tk.IntVar(value = 0)
        self.hand_speed = tk.IntVar(value = 0)
        self.hand_encoder = tk.IntVar(value = 0)

        connection = ttk.Frame(self)   
        status = ttk.Frame(self)
        suuri = ttk.Frame(self)
        shot = ttk.Frame(self)
        pull = ttk.Frame(self)

        self.robot_euler = tk.DoubleVar()
        self.robot_angle = tk.DoubleVar()
        self.robot_speed = tk.DoubleVar()
        self.robot_dir = tk.DoubleVar()

        self.turn_speed = tk.IntVar()

        self.pass_speed = tk.IntVar()
        self.pass_delay = tk.IntVar()
        self.dribble_speed = tk.IntVar()
        self.dribble_delay = tk.IntVar()
        self.shot_speed = tk.IntVar()
        self.shot_delay = tk.IntVar()

        # connection frame
        ttk.Label(connection, text="port: ").grid(row=0, column=0, sticky='EW')
        self.cmb_port = ttk.Combobox(connection, textvariable=self.port, values=[])
        self.cmb_port.grid(row=0, column=1, sticky='EW')
        ttk.Label(connection, text="baudrate: ").grid(row=1, column=0, sticky='EW')
        ttk.Combobox(connection, textvariable=self.baudrate, values=['9600', '19200', '38400', '57800', '115200']).grid(row=1, column=1, sticky='EW')
        ttk.Button(connection, textvariable=self.con,
                   command=self.toggleConnect).grid(row=2, column=0, columnspan=2, sticky='EW', ipadx=20)
        
        connection.grid_columnconfigure(0, weight=0)
        connection.grid_columnconfigure(1, weight=0)
        connection.grid_rowconfigure(0, weight=0, pad=10)
        connection.grid_rowconfigure(1, weight=0, pad=10)
        connection.grid_rowconfigure(2, weight=0, pad=10)

        ttk.Label(status, text="Суурийн төлөв:").grid(row=0, column=0, columnspan=2, sticky='EW', pady=20)
        ttk.Label(status, text="Өнцөг: ").grid(row=1, column=0, sticky='EW')
        ttk.Label(status, textvariable=self.robot_angle).grid(row=1, column=1, sticky='EW')
        ttk.Label(status, text="LPMS өнцөг: ").grid(row=2, column=0, sticky='EW')
        ttk.Label(status, textvariable=self.robot_euler).grid(row=2, column=1, sticky='EW')
        ttk.Label(status, text="Чиглэл: ").grid(row=3, column=0, sticky='EW')
        ttk.Label(status, textvariable=self.robot_dir).grid(row=3, column=1, sticky='EW')
        ttk.Label(status, text="Хурд: ").grid(row=4, column=0, sticky='EW')
        ttk.Label(status, textvariable=self.robot_speed).grid(row=4, column=1, sticky='EW')
        ttk.Label(status, text="Гар энкодер: ").grid(row=5, column=0, sticky='EW')
        ttk.Label(status, textvariable=self.hand_encoder).grid(row=5, column=1, sticky='EW')

        status.grid_columnconfigure(0, weight=0)
        status.grid_columnconfigure(1, weight=0)
        status.grid_rowconfigure(0, weight=0, pad=10)
        status.grid_rowconfigure(1, weight=0, pad=10)
        status.grid_rowconfigure(2, weight=0, pad=10)
        status.grid_rowconfigure(3, weight=0, pad=10)
        status.grid_rowconfigure(4, weight=0, pad=10)

        # scale
        ttk.Label(suuri, text="Коэфициентууд:").grid(row=0, column=0, columnspan=2, sticky='EW')
        ttk.Label(suuri, text="Хурдатгал: ").grid(row=1, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.accel).grid(row=1, column=1, sticky='EW')
        ttk.Label(suuri, text="Эргэх scale: ").grid(row=2, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.rotate_scale).grid(row=2, column=1, sticky='EW')
        ttk.Label(suuri, text="Мотор1: ").grid(row=3, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.m1).grid(row=3, column=1, sticky='EW')
        ttk.Label(suuri, text="Мотор2: ").grid(row=4, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.m2).grid(row=4, column=1, sticky='EW')
        ttk.Label(suuri, text="Мотор3: ").grid(row=5, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.m3).grid(row=5, column=1, sticky='EW')
        ttk.Label(suuri, text="Мотор4: ").grid(row=6, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.m4).grid(row=6, column=1, sticky='EW')
        ttk.Button(suuri, text="Set params", command=self.setParams).grid(row=7, column=0, columnspan=2, sticky='EW')
        # move
        ttk.Label(suuri, text="Суурийн явалт:").grid(row=8, column=0, sticky='EW', pady=(20, 0))
        ttk.Label(suuri, text="Чиглэл: ").grid(row=9, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.direction).grid(row=9, column=1, sticky='EW')
        ttk.Label(suuri, text="Хурд: ").grid(row=10, column=0, sticky='EW')
        ttk.Entry(suuri, textvariable=self.speed).grid(row=10, column=1, sticky='EW')
        ttk.Button(suuri, text="Яв", command=self.move).grid(row=12, column=0, columnspan=2, sticky='EW')
        ttk.Button(suuri, text="Зогс", command=self.stop).grid(row=13, column=0, columnspan=2, sticky='EW')
        # turn
        ttk.Label(suuri, text="Суурийн эргэлт:").grid(row=14, column=0, sticky='EW', pady=(20, 0))
        ttk.Label(suuri, text="Эргэх хурд").grid(row=15, column=0, sticky='EW', padx=10)
        ttk.Entry(suuri, textvariable=self.turn_speed).grid(row=15, column=1, sticky='EW', padx=10)
        ttk.Button(suuri, text="Эргүүлэх", command=self.turn).grid(row=16, column=0, columnspan=2, sticky='EW', padx=10)


        suuri.grid_columnconfigure(0, weight=0)
        suuri.grid_columnconfigure(1, weight=0)
        suuri.grid_rowconfigure(0, weight=0, pad=10)
        suuri.grid_rowconfigure(1, weight=0, pad=10)
        suuri.grid_rowconfigure(2, weight=0, pad=10)
        suuri.grid_rowconfigure(3, weight=0, pad=10)
        suuri.grid_rowconfigure(4, weight=0, pad=10)
        suuri.grid_rowconfigure(5, weight=0, pad=10)
        suuri.grid_rowconfigure(6, weight=0, pad=10)
        suuri.grid_rowconfigure(7, weight=0, pad=10)
        suuri.grid_rowconfigure(8, weight=0, pad=10)
        suuri.grid_rowconfigure(9, weight=0, pad=10)
        suuri.grid_rowconfigure(10, weight=0, pad=10)
        suuri.grid_rowconfigure(11, weight=0, pad=10)
        suuri.grid_rowconfigure(12, weight=0, pad=10)
        suuri.grid_rowconfigure(13, weight=0, pad=10)
        suuri.grid_rowconfigure(14, weight=0, pad=10)
        suuri.grid_rowconfigure(15, weight=0, pad=10)
        suuri.grid_rowconfigure(16, weight=0, pad=10)

        ttk.Label(shot, text="Шидэх механизм:").grid(row=0, column=0, sticky='EW')
        # pass
        ttk.Label(shot, text="Дамжуулах").grid(row=1, column=0, sticky='EW')
        ttk.Entry(shot, textvariable=self.pass_speed, width=7).grid(row=1, column=1, sticky='EW', padx=5)
        ttk.Entry(shot, textvariable=self.pass_delay, width=7).grid(row=1, column=2, sticky='EW', padx=5)
        ttk.Button(shot, text="set", command=self.pass_ball).grid(row=1, column=3, sticky='EW', padx=5)
        # dribble
        ttk.Label(shot, text="Залах").grid(row=2, column=0, sticky='EW')
        ttk.Entry(shot, textvariable=self.dribble_speed, width=7).grid(row=2, column=1, sticky='EW', padx=5)
        ttk.Entry(shot, textvariable=self.dribble_delay, width=7).grid(row=2, column=2, sticky='EW', padx=5)
        ttk.Button(shot, text="set", command=self.dribble_ball).grid(row=2, column=3, sticky='EW', padx=5)
        # shot
        ttk.Label(shot, text="Шидэх").grid(row=3, column=0, sticky='EW')
        ttk.Entry(shot, textvariable=self.shot_speed, width=7).grid(row=3, column=1, sticky='EW', padx=5)
        ttk.Entry(shot, textvariable=self.shot_delay, width=7).grid(row=3, column=2, sticky='EW', padx=5)
        ttk.Button(shot, text="set", command=self.shot_ball).grid(row=3, column=3, sticky='EW', padx=5)
        # stop
        ttk.Button(shot, text="stop", command=self.stop_shot).grid(row=4, column=3, sticky='EW', padx=5)

        shot.grid_columnconfigure(0, weight=0)
        shot.grid_columnconfigure(1, weight=0)
        shot.grid_columnconfigure(2, weight=0)
        shot.grid_columnconfigure(3, weight=0)
        shot.grid_rowconfigure(0, weight=0, pad=10)
        shot.grid_rowconfigure(1, weight=0, pad=10)
        shot.grid_rowconfigure(2, weight=0, pad=10)
        shot.grid_rowconfigure(3, weight=0, pad=10)

        ttk.Label(pull, text="Татах механизм").grid(row=0, column=0, sticky='EW', pady=(20, 0))
        ttk.Label(pull, text="Эргэх хурд").grid(row=1, column=0, sticky='EW')
        ttk.Entry(pull, textvariable=self.hand_speed, width=10).grid(row=1, column=1, sticky='EW', padx=10)
        ttk.Button(pull, text="Татах", command=self.pullHand).grid(row=1, column=2, sticky='EW')
        ttk.Button(pull, text="Буцаах", command=self.pushHand).grid(row=2, column=2, sticky='EW')
        ttk.Button(pull, text="Зогсоох", command=self.stopHand).grid(row=3, column=2, sticky='EW')

        pull.grid_columnconfigure(0, weight=0)
        pull.grid_columnconfigure(1, weight=0)
        pull.grid_columnconfigure(2, weight=0)
        pull.grid_rowconfigure(0, weight=0, pad=10)
        pull.grid_rowconfigure(1, weight=0, pad=10)
        pull.grid_rowconfigure(2, weight=0, pad=10)
        pull.grid_rowconfigure(3, weight=0, pad=10)

        
        connection.grid(row=0, column=0, sticky='NSEW')
        status.grid(row=1, column=0, sticky='NSEW')
        suuri.grid(row=0, rowspan=2, column=1, sticky='NSEW', padx=30)
        shot.grid(row=0, column=2, sticky='NSEW', padx=30)
        pull.grid(row=1, column=2, sticky='NSEW', padx=30)


        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=0)
        self.grid_columnconfigure(2, weight=0)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=1)
    

        self.update_ports()

    def turn(self):
        speed = self.turn_speed.get()
        command=f"turn,pwm={speed}\n"
        self.ser.write(command.encode())

    def pass_ball(self):
        speed = self.pass_speed.get()
        delay = self.pass_delay.get()
        command=f"pass,pwm={speed},time={delay}\n"
        self.ser.write(command.encode())

    def shot_ball(self):
        speed = self.shot_speed.get()
        delay = self.shot_delay.get()
        command=f"shot,pwm={speed},time={delay}\n"
        self.ser.write(command.encode())

    def dribble_ball(self):
        speed = self.dribble_speed.get()
        delay = self.dribble_delay.get()
        command=f"dribble,pwm={speed},time={delay}\n"
        self.ser.write(command.encode())

    def stop_shot(self):
        command=f"cancel\n"
        self.ser.write(command.encode())

    def pushHand(self):
        speed = self.hand_speed.get()
        command=f"hand,pwm={-speed}\n"
        self.ser.write(command.encode())

    def pullHand(self):
        speed = self.hand_speed.get()
        command=f"hand,pwm={speed}\n"
        self.ser.write(command.encode())

    def stopHand(self):
        command = "hand,pwm=0\n"
        self.ser.write(command.encode())

    def setParams(self):
        m1 = self.m1.get()
        m2 = self.m2.get()
        m3 = self.m3.get()
        m4 = self.m4.get()
        accel = self.accel.get()
        rotate_scale = self.rotate_scale.get()
        command = f"scale,m1={m1},m2={m2},m3={m3},m4={m4},accel={accel},scale={rotate_scale}\n"
        self.ser.write(command.encode())

    def stop(self):
        command = "stop,\n"
        self.ser.write(command.encode())

    def move(self):
        dir = self.direction.get()
        speed = self.speed.get()
        angle = self.angle.get()
        command = f"move,dir={dir},speed={speed},angle={angle}\n"
        self.ser.write(command.encode())

    def read(self):
        while not self.stop_event.is_set():
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    if line.startswith("log"):
                        print(line)
                    else:
                        parts = line.split(",")
                        for part in parts:
                            if "=" in part:
                                key, value = part.split("=")
                                key = key.strip()
                                value = value.strip()
                                if key == 'angle':
                                    self.robot_angle.set(float(value))
                                elif key == 'euler':
                                    self.robot_euler.set(float(value))
                                elif key == 'speed':
                                    self.robot_speed.set(float(value))
                                elif key == 'dir':
                                    self.robot_dir.set(float(value))
                                elif key == 'enc':
                                    self.hand_encoder.set(int(value))
                except Exception as ex:
                    print("Error: ", ex)

    def connect(self):
        try:
            port = self.port.get()
            baudrate = self.baudrate.get()
            if not port or not baudrate:
                messagebox.showerror("Error", "Port болон baudrate - ээ оруулна уу!")
                return
            self.ser.port = port
            self.ser.baudrate = baudrate
            self.ser.open()
            if self.ser.is_open:
                self.con.set("Disconnect")
                self.stop_event.clear()
                self.thread = threading.Thread(target=self.read, daemon=True)
                self.thread.start()
                
        except Exception as ex:
            messagebox.showerror("Serial connection: ", ex)

    def disconnect(self):
        if self.ser.is_open:
            self.stop_event.set()
            self.thread.join(timeout=1)
            self.ser.close()
            self.con.set('Connect')

    def toggleConnect(self):
        if self.ser.is_open:
            self.disconnect()
        else:
            self.connect()
            
    def update_ports(self):
        if self.ser.is_open:
            return

        self.cmb_port['values'] = ()

        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]

        if not port_list:
            port_list = ['No port found']
        
        self.cmb_port['values'] = port_list

        self.after(5000, self.update_ports)


root = Main()
root.mainloop()
