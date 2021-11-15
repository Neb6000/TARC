import math
import serial
import time
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

TIMESTEP = 0.01 #s
GRAVITY = -9.80 # m/s/s

class Plotter:
    def __init__(self, minValue, maxValue, fig):
        self.x_len = 1200
        self.y_range = [minValue, maxValue]
        self.ax = fig
        self.xs = list(range(0, 1200))
        self.ys = [0] * self.x_len
        self.ax.set_ylim(self.y_range)
        self.line, = self.ax.plot(self.xs, self.ys)
    def return_line(self):
        return self.line
    def update(self, value):
        self.ys.append(value)
        # Limit y list to set number of items
        self.ys = self.ys[-self.x_len:]
        # Update line with new Y values
        self.line.set_ydata(self.ys)
        return self.line

fig = plt.figure()
plot1 = Plotter(-10, 400, fig.add_subplot(3,1,1))
plot2 = Plotter(-100, 110, fig.add_subplot(3,1,2))
plot3 = Plotter(-10, 100, fig.add_subplot(3,1,3))
#plot4 = Plotter(-100, 100, fig.add_subplot(3,2,2))
#plot5 = Plotter(-100, 100, fig.add_subplot(3,2,4))
#plot6 = Plotter(-100, 100, fig.add_subplot(3,2,6))
#line = [plot1.return_line(), plot2.return_line(), plot3.return_line(), plot4.return_line(), plot5.return_line(), plot6.return_line()]
line = [plot1.return_line(), plot2.return_line(), plot3.return_line()]

arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)

mass = 0.650 # kg

force = 0
altitude = 0
velocity = 0
deploy_angle = 0
def calc_drag():
    a = -0.00000407762
    b = 1.54521
    c = -0.0009436
    cd = a * pow(deploy_angle, b) + c
    return cd * pow(velocity, 2)

current_time = 0
delta_time = 0
previous_time = 0
def timekeeper():
    global current_time, delta_time, previous_time
    current_time = time.time()
    delta_time = current_time - previous_time
    previous_time = current_time


state = 0

burn_begins = 0
begins = 0
wait_for = 5 # s
def wait_for_liftoff():
    global state, burn_begins, altitude, velocity
    altitude = 0
    velocity = 0
    if current_time - begins > wait_for:
        state += 1
        burn_begins = current_time
        print(state)

def initialize():
    global begins, state
    begins = current_time
    state += 1
    print(state)

burn_for = 1.45 # s
def motor_burn():
    global state, force
    if current_time - burn_begins < burn_for:
        force += 45.7 # N
    else:
        state += 1
        print(state)

def send_data():
    global deploy_angle
    
    #arduino.write(bytes(altitude, 'utf-8'))
    send = str(round(altitude, 1)) + ":"
    arduino.write(bytes(send, 'utf-8'))
    #arduino.write(bytes('\t', 'utf-8'))
    #arduino.flush()
    if arduino.in_waiting > 0:
        data = arduino.readline()
        try:
            deploy_angle = float(data)
            
            #print(deploy_angle)
            #print(data)
            #print(velocity)
            #print(send)
            arduino.reset_input_buffer()
        except:
            print("bad transmission") 
        #deploy_angle = float(arduino.readline())
        #print(deploy_angle)
max_alt = 0
def physics():
    global altitude, velocity, force, max_alt
    force += mass * GRAVITY + calc_drag()
    velocity += (force / mass) * delta_time
    altitude += velocity * delta_time
    if altitude > max_alt:
        max_alt = altitude
    #print(altitude)
    force = 0

def update(i):
    global line, max_alt
    #time.sleep(0.05)
    timekeeper()
    if state == 0:
        initialize()
    if state == 1:
        wait_for_liftoff()
    elif state == 2:
        motor_burn()
    elif state == 3:
        if altitude < 0:
            print(max_alt)
            x = 1992/0
    send_data()
    physics() # should always be last thing called

    line[0] = plot1.update(altitude)
    line[1] = plot2.update(velocity)
    line[2] = plot3.update(deploy_angle) 

    return line

ani = animation. FuncAnimation(fig, 
    update,
    interval=10,
    blit=True)

plt.show()