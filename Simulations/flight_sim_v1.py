import math
import numpy as np
from numpy.polynomial.polynomial import Polynomial

TIMESTEP = 0.01 #s
MASS = 0.650 # kg
GRAVITY = -9.80 # m/s/s
TARGET_APOGEE = 254.508 # m

MIN_CD = -10 #0.000944
MAX_CD = 10 #0.00521

MAX_ERROR = 1 # m

def apogee_finder(velocity, altitude, cd):
    #time = 0
    weight = MASS * GRAVITY
    while velocity > 0:
        print(velocity)
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
        print(cd)
        alt = apogee_finder(velocity, altitude, cd)
        print(alt)
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

MAX_ALT = 350 # m
DISCRETE_ALT = 1 # m
MAX_SPEED = 80 # m/s
DISCRET_SPEED = 0.3 # m/s

data = np.zeros((int(MAX_ALT/DISCRETE_ALT), int(MAX_SPEED/DISCRET_SPEED))) # nightmare shitty code
regression1 = np.empty((int(MAX_ALT/DISCRETE_ALT)))
alt = 0
for i in data:
    speed = 2*DISCRET_SPEED
    alt += DISCRETE_ALT
    x = np.zeros(len(i))
    print(alt)
    for j in i:
        j = cd_finder(speed, alt)
        speed += DISCRET_SPEED
        x[speed/DISCRET_SPEED] = speed
    p = Polynomial((1,2,3))
    regression1[alt/DISCRETE_ALT] = p.fit(x, i)


print(regression1[0].convert().coef)
print(regression1[0])
regression2 = np.empty(3)


#print(cd_finder(84, 43))