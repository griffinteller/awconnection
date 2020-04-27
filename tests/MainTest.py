from awconnection import RobotConnection
import matplotlib.pyplot as plt
import time
import copy

r = RobotConnection()
r.connect()

r.set_tire_torque("BackLeft", 1000)
r.set_tire_torque("BackRight", 1000)
r.set_tire_torque("FrontLeft", 1000)
r.set_tire_torque("FrontRight", 1000)

r.disconnect()
