'''
Utility functions for ABR robot programming.
'''

import numpy as np
from time import time, sleep

# convert an AW x/y/z vector into a 3x1 numpy vector
def xyz_vec(v):
    if type(v) is dict:
        return np.array([v['x'], v['y'], v['z']])
    else:
        return np.array([v.x, v.y, v.z])
# end method xyz_vec

# designed for loops targeting a fixed iteration/sampling rate
# given a cycle start time and a target cycle_duration, sleep until
# the end of the cycle
def sleep_until_end_of_cycle(cycle_start_time, cycle_duration):
    t = time()
    cycle_time_left = cycle_duration - (t - cycle_start_time)
    if cycle_time_left > 0:
        sleep(cycle_time_left)
    return cycle_time_left
# end sleep_until_end_of_cycle

def add_bearings(bearing, delta):
    bearing += delta
    if bearing > 180:
        bearing -= 360
    elif bearing < -180:
        bearing += 360
    return bearing
# end function add_bearings
