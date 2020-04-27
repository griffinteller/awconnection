from awconnection import RobotConnection
bot = RobotConnection()
bot.connect()
forward = bot.info.gyroscope.forward
right = bot.info.gyroscope.right
up = bot.info.gyroscope.up
print(forward.x, forward.y, forward.z)
print(right.x, right.y, right.z)
print(up.x, up.y, up.z)
bot.disconnect()

exit(0)

######################################################################

from awconnection import RobotConnection
bot = RobotConnection()
bot.connect()
bot.set_tire_steering("FrontLeft", -10)
bot.set_tire_steering("FrontRight", -10)
bot.set_tire_torque("FrontLeft", 5000)
bot.set_tire_torque("FrontRight", 5000)
bot.disconnect()

exit(0)

#######################################################################

from awconnection import RobotConnection
import numpy as np
from time import sleep

bot = RobotConnection()
bot.connect()

'''
bot.info.gyroscope.forward # up, right
bot.info.gps.position
bot.info.lidar.distance_matrix  # 2FIX: what happened to range
bot.info.altimeter.altitude
bot.info.gamemode
bot.info.isIt
bot.info.radar.it_ping  # pings
bot.info.timestamp
'''

dm1 = np.array(bot.info.lidar.distance_matrix)
sleep(1)
dm2 = np.array(bot.info.lidar.distance_matrix)
sleep(1)
dm3 = np.array(bot.info.lidar.distance_matrix)

dms = [dm1, dm2, dm3]

mean_dm = sum(dms) / len(dms)

print(mean_dm.shape)
print(dm1[5, 0], dm2[5, 0], dm3[5, 0], mean_dm[5, 0])

bot.disconnect()
