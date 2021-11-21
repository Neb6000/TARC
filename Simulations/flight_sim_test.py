import math
import serial
import time
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

TIMESTEP = 0.01 #s
MASS = 0.650 # kg
GRAVITY = -9.80 # m/s/s
TARGET_APOGEE = 254.508 # m

MIN_CD = 0.000944
MAX_CD = 0.00521

MAX_ERROR = 1 # m

DELAY = 0.9 # s, time between commanded angle and actual angle

def apogee_finder(velocity, altitude, cd):
    #time = 0
    weight = MASS * GRAVITY
    while velocity > 0:
        drag_force = -math.pow(velocity, 2) * cd
        acceleration = (drag_force + weight)/MASS #* TIMESTEP
        altitude += velocity * TIMESTEP
        velocity += acceleration * TIMESTEP

    return altitude


def alt_finder(alt):
    a = 0 #-0.05
    return a * (alt - TARGET_APOGEE) + TARGET_APOGEE

def cd_finder(velocity, altitude):
    max_cd = MAX_CD
    min_cd = MIN_CD
    i = 0
    if apogee_finder(velocity, altitude, min_cd) < alt_finder(altitude):
        return min_cd
    elif apogee_finder(velocity, altitude, max_cd) > alt_finder(altitude):
        return max_cd

    while True:
        i += 1
        
        cd = (min_cd + max_cd)/2
        #print(cd)
        alt = apogee_finder(velocity, altitude, cd)
        #print(alt)
        if abs(alt-alt_finder(altitude)) < MAX_ERROR:
            print(i)
            return cd
        if alt < alt_finder(altitude):
            max_cd = cd
        else:
            min_cd = cd

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

#arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)

mass = 0.650 # kg

force = 0
altitude = 0
velocity = 0
deploy_angle = [0]
deploy_time = [0]
commanded_angle = 0
actual_angle = 0
def calc_drag():
    a = -0.00000407762
    b = 1.54521
    c = -0.0009436
    cd = a * pow(actual_angle, b) + c
    return cd * pow(velocity, 2)

current_time = 0
delta_time = 0.01
previous_time = 0
def timekeeper():
    global current_time, delta_time, previous_time
    current_time += delta_time
    #delta_time = current_time - previous_time
    previous_time = current_time


state = 0

burn_begins = 0
begins = 0
wait_for = 0.1 # s
def wait_for_liftoff():
    global state, burn_begins, altitude, velocity
    altitude = 0
    velocity = 0
    if current_time - begins > wait_for:
        state += 1
        burn_begins = current_time
        print(state)

def initialize():
    global begins, state, deploy_time
    begins = current_time
    deploy_time[0] = current_time
    state += 1
    print(state)

burn_for = 1.45 # s
def motor_burn():
    global state, force
    if current_time - burn_begins < burn_for:
        force += 45.9 # N
    else:
        state += 1
        print(state)


def angle_fuzzer():
    global commanded_angle, actual_angle
    if current_time - deploy_time[0] > DELAY:
        commanded_angle = deploy_angle[0]
        deploy_angle.pop(0)
        deploy_time.pop(0)
    actual_angle = commanded_angle

def active_control():
    global deploy_angle
    a = -0.00000407762
    b = 1.54521
    c = -0.0009436
    cd = -cd_finder(velocity, altitude)
    dep = pow((cd - c) / a, 1/b)
    dep_angle = 0
    if dep < 0:
        dep_angle = 0
    elif dep > 90:
        dep_angle = 90
    else:
        dep_angle = dep
    deploy_angle.append(dep_angle)
    deploy_time.append(current_time)
    angle_fuzzer()


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
        active_control()
        if velocity < 0:
            print(max_alt)
            x = 231231/0
    #angle_fuzzer()
    physics() # should always be last thing called

    line[0] = plot1.update(altitude)
    line[1] = plot2.update(velocity)
    line[2] = plot3.update(actual_angle) 

    return line

ani = animation. FuncAnimation(fig, 
    update,
    interval=1,
    blit=True)

plt.show()