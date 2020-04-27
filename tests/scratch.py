'''
a script for scratch work
'''

from matplotlib import pyplot as plt
import math
import numpy as np



























### TEST BEARING
# good
def bearing(x, z):
    znorm = z / math.sqrt(x ** 2 + z ** 2)
    b = 360 * math.acos(znorm) / (2 * math.pi)
    if x < 0:
        b *= -1
    return b
# end function bearing
'''
degrees = [a for a in range(360)]
rads = [2 * math.pi * d / 360 for d in degrees]
z = [math.cos(a) for a in rads]
x = [math.sin(a) for a in rads]
bearings = [bearing(val[0], val[1]) for val in zip(x, z)]

plt.plot(degrees, bearings)
plt.show()
'''

### TEST PITCH
# good
def pitch(forward_y):
    return 360 * math.asin(forward_y) / (2 * math.pi)
# end function pitch
'''
fy = np.linspace(-1, 1, 100)
pitches = [pitch(val) for val in fy]
plt.plot(fy, pitches)
plt.show()
'''


### TEST ROLL
# good
def roll(right_y):
    return -360 * math.asin(right_y) / (2 * math.pi)
ry = np.linspace(-1, 1, 100)
rolls = [roll(val) for val in ry]
plt.plot(ry, rolls)
plt.show()
