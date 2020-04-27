from tests.ZRobot import ZRobot
from tests.ControlDashboard import ControlDashboard
from tests.aw_utils import sleep_until_end_of_cycle, add_bearings
from time import time, sleep
import numpy as np
from random import randint

bot = ZRobot()
dash = ControlDashboard(bot)

### general stuff
RUN_CONTROL_DASHBOARD = True
DEBUG_TATTLES = False

### stuff for test mode
TEST_MODE = True
CHASING = True
# target positions for testing (note: y coordinate is irrelevant here)
bot.state['tgt_pos'] = np.array([-400, 0, -300])
#bot.state['tgt_pos'] = np.array([300, 0, -350])
#bot.state['tgt_pos'] = np.array([100, 0, 600])
#bot.state['tgt_pos'] = np.array([0, 0, 600])
#bot.state['tgt_pos'] = np.array([0, 0, -50])

### stuff for live mode
# for roll-based course correction
bot.state['roll_course_correction'] = 0
# for checking stuckness
bot.state['stuck'] = False
bot.state['stuck_timeout'] = 8
bot.state['last_moved_time'] = time()
# for too-steep hills
bot.state['too_steep'] = False
bot.state['uphill_timeout'] = 8
# head downhill if grade is consistently above this threshold
bot.state['uphill_grade_threshold'] = 90
bot.state['last_level_time'] = time()
bot.state['last_steep_time'] = time()
bot.state['heading_downhill'] = False
# stop heading downhill when grade is consistently closer to zero than this threshold
bot.state['downhill_grade_threshold'] = 90
# for moving downhill
bot.state['moving_downhill'] = False
bot.state['moving_downhill_time'] = time()
bot.state['moving_downhill_timeout'] = 4

bot.state['tgt_pos_delta'] = 0


# main control loop
cycle_duration = 0.05  # 50 milliseconds -> 20 Hz
dash_time = time()
while True:
    if DEBUG_TATTLES:
        print('########## STARTING MAIN CYCLE LOOP ##########')

    cycle_start_time = time()
    bot.sensor_lock.acquire()


    if TEST_MODE:
        bot.state['tgt_pos_delta'] = bot.state['tgt_pos'] - bot.position
        bot_is_it = CHASING
    else:
        bot_is_it = bot.is_it
        if bot_is_it:
            # 2FIX: RIGHT NOW I'M CHASING THE FIRST PING IN THE LIST.  THIS LOGIC WILL NEED
            # IMPROVEMENT WHEN THERE ARE MULTIPLE OPPONENTS
            bot.state['tgt_pos_delta'] = bot.pings[0]
            if DEBUG_TATTLES:
                print('chasing...')
        else:
            bot.state['tgt_pos_delta'] = bot.it_ping
            if DEBUG_TATTLES:
                print('fleeing...')

    if bot_is_it:
        bot.chase()
    else:
        bot.flee()

    # only run control dashboard once per second (it's slow)
    if RUN_CONTROL_DASHBOARD:
        new_dash_time = time()
        if new_dash_time - dash_time > 1:
            dash_time = new_dash_time
            #t0 = time()
            dash.update()
            #print(1000 * (time() - t0))

    bot.sensor_lock.release()
    sleep_until_end_of_cycle(cycle_start_time, cycle_duration)
# end while True
