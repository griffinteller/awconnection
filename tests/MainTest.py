from awconnection import RobotConnection
import matplotlib.pyplot as plt
import time
import copy

r = RobotConnection()
r.connect()

r.set_tire_steering("BackLeft", 0)
r.set_tire_steering("BackRight", 0)
r.set_tire_steering("FrontLeft", -20)
r.set_tire_steering("FrontRight", 20)

r.disconnect()
