from tests.ZRobot import ZRobot
from tests.ControlDashboard import ControlDashboard
from tests.aw_utils import sleep_until_end_of_cycle, add_bearings
from time import time, sleep
import numpy as np
from random import randint

# steer the robot toward its target position
def set_steering(bot):

    # randomly change bearing -- slowly if level, faster if on a steeper slope
    tgt_bearing_delta = 0.1 * abs(bot.smoothed_pitch) * randint(-1, 1)
    if tgt_bearing_delta != 0:
        bot.state['tgt_bearing'] = add_bearings(bot.state['tgt_bearing'], tgt_bearing_delta)
        print('new bearing: {}'.format(bot.state['tgt_bearing']))
    tgt_bearing = bot.state['tgt_bearing']
    current_bearing = bot.smoothed_bearing

    bearing_delta = bot.calculate_yaw_delta(current_bearing, tgt_bearing)
    steering_angle = 0.1 * bearing_delta  # ?????????

    ### apply steering angle to robot
    bot.steering_angle = steering_angle
# end function set_steering