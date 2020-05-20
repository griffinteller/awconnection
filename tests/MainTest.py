from awconnection import RobotConnection
import time

r = RobotConnection()
r.connect(1)


"""def set_steering(bearing):

    r.set_tire_steering("1", bearing)
    r.set_tire_steering("2", bearing)"""


def set_four_wheel_torque(torque):

    r.set_tire_torque("1", torque)
    r.set_tire_torque("3", torque)
    r.set_tire_torque("2", -torque)
    r.set_tire_torque("4", -torque)

i = 1
while True:

    r.set_tire_steering("1", 10 * i)
    r.set_tire_steering("2", 10 * i)
    r.set_tire_steering("3", 0)
    r.set_tire_steering("4", 0)
    i*=-1
    time.sleep(0.3)

r.disconnect()
