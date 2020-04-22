from awconnection import RobotConnection
import matplotlib.pyplot as plt
import time

r = RobotConnection()
r.connect()

r.set_tire_torque("BackLeft", 300)
r.set_tire_torque("BackRight", 300)

while True:

    plt.pcolormesh(r.info.lidar.distance_matrix)
    plt.gca().invert_yaxis()
    plt.colorbar()
    plt.show()
    time.sleep(0.5)

r.disconnect()