import math


TIMESTEP = 0.01 #s
MASS = 0.650 # kg
GRAVITY = -9.80 # m/s/s
TARGET_APOGEE = 254.508 # m

MIN_CD = 0.000944
MAX_CD = 0.00521

MAX_ERROR = 1 # m

def apogee_finder(velocity, altitude, cd):
    #time = 0
    weight = MASS * GRAVITY
    while velocity > 0:
        drag_force = -math.pow(velocity, 2) * cd
        acceleration = (drag_force + weight)/MASS #* TIMESTEP
        altitude += velocity * TIMESTEP
        velocity += acceleration * TIMESTEP

    return altitude



def cd_finder(velocity, altitude):
    max_cd = MAX_CD
    min_cd = MIN_CD
    i = 0
    while True:
        i += 1
        cd = (min_cd + max_cd)/2
        alt = apogee_finder(velocity, altitude, cd)
        #print(alt)
        if abs(alt-TARGET_APOGEE) < MAX_ERROR:
            print(i)
            return cd
        if alt < TARGET_APOGEE:
            max_cd = cd
        else:
            min_cd = cd

time =  0
velocity = 0

#print(apogee_finder(90, 50, 0))

print(cd_finder(84, 43))