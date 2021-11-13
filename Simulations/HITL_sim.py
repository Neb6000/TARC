import math
import serial
import time

TIMESTEP = 0.01 #s
GRAVITY = -9.80 # m/s/s

arduino = serial.Serial(port='COM4', baudrate=230400, timeout=.1)

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
wait_for = 15 # s
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
        force += 54.7 # N
    else:
        state += 1
        print(state)

def send_data():
    global deploy_angle
    #arduino.write(bytes(altitude, 'utf-8'))
    send = str(round(altitude, 3))
    arduino.write(bytes(send, 'utf-8'))
    arduino.write(bytes('\t', 'utf-8'))
    arduino.flush()
    if arduino.in_waiting > 0:
        data = arduino.readline()
        try:
            #deploy_angle = float(data)
            #print(deploy_angle)
            print(data)
           # print(velocity)
            #print(altitude)
            
        except:
            print("bad transmission") 
        #deploy_angle = float(arduino.readline())
        #print(deploy_angle)

def physics():
    global altitude, velocity, force
    force += mass * GRAVITY + calc_drag()
    velocity += (force / mass) * delta_time
    altitude += velocity * delta_time
    #print(altitude)
    force = 0

while True:
    time.sleep(0.05)
    timekeeper()
    if state == 0:
        initialize()
    if state == 1:
        wait_for_liftoff()
    elif state == 2:
        motor_burn()
    elif state == 3:
        if altitude < 0:
            break
    send_data()
    physics() # should always be last thing called

