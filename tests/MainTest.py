from awconnection import RobotConnection
import matplotlib.pyplot as plt
import time
import copy

r = RobotConnection()
r.connect()

r.set_tire_torque("BackLeft", 5000)
r.set_tire_torque("BackRight", 5000)
r.set_tire_torque("FrontLeft", 5000)
r.set_tire_torque("FrontRight", 5000)

time.sleep(3)

while True:

    r.set_tire_steering("BackLeft", 0)
    r.set_tire_steering("BackRight", 0)
    r.set_tire_steering("FrontLeft", 20)
    r.set_tire_steering("FrontRight", 20)

r.disconnect()