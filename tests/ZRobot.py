'''
ZRobot is my robot class -- a wrapper class for the abrconnection RobotConnection class,
with more higher-level functionality.
'''

from awconnection import RobotConnection
import math
from threading import Thread, Lock
from tests.aw_utils import xyz_vec, sleep_until_end_of_cycle, add_bearings
from time import time, sleep
import numpy as np
from copy import deepcopy

# 2FIX: GET RID OF THIS
DEBUG_TATTLES = False





# 2FIX: FACTOR OUT TAG LOGIC



'''
Notes

we do synchronization locking for the entirety of each iteration of various high-level
cycle-loops, including update_sensors.  this way, all prior/last/current sensor values are
always consistent within a given cycle-loop iteration

all high-level cycle-loops should have 2 things:
* sensor locking for the entirety of each iteration of the loop
*  a call to sleep_until_end_of_cycle at the end
'''

class ZRobot():

    def __init__(self):
        try:
            self.bot = RobotConnection()
            self.bot.connect()
            print('ZRobot: successfully connected to robot')
        except Exception as e:
            print('ZRobot: failed to connect to robot')
            print(e)

        # various things that iterate and sample the sensors, including the actual updating of the
        # sensors themselves, all run at the same frequency.  cycle time of 40 milliseconds -> 25 Hz
        self.cycle_duration = 0.04

        # how old can the latest sensor info be before we call it stale?
        self.max_sensor_staleness = 0.1  # 2 cycles
        self.sensors_are_stale = True
        # time of robot instantiation (this will be in seconds since Jan 1, 1970, from the first
        # sensor update)
        self.start_time = time() # time since epoch, in seconds

        # how many trailing sensor results to keep around (set here to ~1 second's worth, i.e., 20)
        self.keep_num_sensors = round(1 / self.cycle_duration)

        # queue for history of sensors and sensor-derived values
        # note: lists are slow for this, but the queue is probably small enough that it doesn't matter (?)
        self.sensor_history = list()

        # set a max absolute torque (ABR system may enforce its own max torque at some point)
        self.max_abs_torque = 4000  # 3000
        # this is a very sharp turn (since we're steering with front and back wheels), and
        # the robot will regularly lose traction turning this hard at high speed
        self.max_abs_steering_angle = 20

        # these are marked as private to distinguish them from their property names
        self._steering_angle = 0
        self._torque = 0

        # number of most recent samples to use for smoothing (of speed, acceleration, etc.)
        self._num_smoothing_samples = 5

        # update_sensors runs in a separate thread
        self.update_sensor_thread = Thread(name='update_sensors', target=self.update_sensors)
        self.sensor_lock = Lock()  # for sensor read/write synchronization across threads
        self.running = True  # used in disconnect method to terminate loop in update-sensor thread
        self.update_sensor_thread.start()

        # catch-all dictionary for any/all state variables
        self.state = dict()

        # make sure sensor history is fully populated before returning
        # (given the setting above for history size, this should take ~1 second)
        t0 = time()
        t1 = t0
        while not self.all_sensors_good:
            t2 = time()
            if t2 - t1 > 10:
                t1 = t2
                print('ZRobot: sensors not populated after {} seconds...'.format(round(t2 - t0)))
            sleep(self.cycle_duration)
    # end constructor

    # couldn't put the call to bot.disconnect() in a destructor, because the underlying robot
    # connection has its own threads open, and the destructor isn't called until all threads
    # are finished (at least in the simple case where you create the robot at the top level
    # of a script)
    # so, calling code needs to call disconnect on the ZRobot at the end of its lifetime (not
    # that I'm aware of anything terrible that happens if you don't...)
    def disconnect(self):
        self.running = False  # terminate loop in update-sensor thread
        try:
            self.bot.disconnect()
            print('ZRobot: successfully disconnected from robot')
        except Exception as e:
            print('ZRobot: failed to disconnect from robot')
            print(e)
    # end function disconnect



    ########## PROPERTIES



    # handle all sensor-derived properties the same way
    # a named property (e.g., 'speed') returns the most recent value for that name from the sensor history
    def __getattr__(self, name):
        try:
            return self.sensor_history[-1][name]
        except:
            AttributeError('"{}" is not a recognized attribute'.format(name))
    # end method __getattr__

    # return boolean -- true if last n sensors in history are fully populated
    def get_last_n_sensors_good(self, n):
        if n > self.keep_num_sensors:
            #print('asked for population of {} sensors, but only keeping {} sensors'.format(n, self.keep_num_sensors))
            return False

        if len(self.sensor_history) < n:
            #print('asked for population of {} sensors, but only have {} sensors so far'.format(n, len(self.sensor_history)))
            return False

        # check that a 2nd-order derivative (which needs the most history) is populated in the
        # oldest sensor
        acceleration = self.sensor_history[-n]['acceleration']
        if acceleration is None:
            #print('acceleration at sensor history position {} is None'.format(n))
            return False

        return True
    # end method get_last_n_sensors_good

    def get_all_sensors_good(self):
        #print('waiting for all {} sensors to be populated...'.format(self.keep_num_sensors))
        return self.get_last_n_sensors_good(self.keep_num_sensors)
    all_sensors_good = property(fget=get_all_sensors_good)

    def get_num_smoothing_samples(self):
        return self._num_smoothing_samples

    def set_num_smoothing_samples(self, n):
        if n < 1:
            raise ValueError('n must be positive integer')
        if n > self.keep_num_sensors:
            raise ValueError("n can't be greater than keep_num_sensors")
        self._num_smoothing_samples = n
    num_smoothing_samples = property(fget=get_num_smoothing_samples, fset=set_num_smoothing_samples)

    def get_steering_angle(self):
        return self._steering_angle
    def set_steering_angle(self, angle):
        # apply absolute limit
        steering_angle = np.sign(angle) * min(abs(angle), self.max_abs_steering_angle)

        self._steering_angle = steering_angle
        self.set_bot_steering(self._steering_angle)  # actually set steering of the robot
    steering_angle = property(fget=get_steering_angle, fset=set_steering_angle)

    def get_torque(self):
        return self._torque
    def set_torque(self, torque):
        torque = np.sign(torque) * min(abs(torque), self.max_abs_torque)  # apply absolute limit
        self._torque = torque
        self.set_bot_torque(self._torque)  # actually set torque of the robot
    torque = property(fget=get_torque, fset=set_torque)

    def get_facing_downhill(self, bearing_tolerance=30):
        bearing = self.smoothed_bearing
        downhill_bearing = self.bearing_for_vec(self.up)
        if bearing is None or downhill_bearing is None:
            return False
        bearing_delta = self.calculate_yaw_delta(bearing, downhill_bearing)
        return abs(bearing_delta) < bearing_tolerance
    # end method get_facing_downhill
    facing_downhill = property(fget=get_facing_downhill)

    def get_is_it(self):
        return self.sensor_history[-1]['is_it']
    is_it = property(fget=get_is_it)



    ########## METHODS



    # given a vector, return the bearing for that vector
    # how bearing is defined:
    #  * bearings are in [-180, 180]
    #  * North (positive z axis) is bearing 0
    #  * East bearings are positive
    #  * West bearings are negative
    #  * y (vertical) is ignored
    #  * if vector is close to vertical (abs(y) > 0.99), bearing is None
    def bearing_for_vec(self, vec):
        vec_x = vec[0]
        vec_y = vec[1]
        vec_z = vec[2]
        if (abs(vec_y) / np.linalg.norm(vec)) > 0.9999:  # ~0.81 degrees from vertical
            return None  # vertical -- no bearing
        else:
            # project forward vector into x-z plane (so, drop y-component, use renormalized z-component)
            bearing_z = vec_z / math.sqrt(vec_x ** 2 + vec_z ** 2)
            bearing = 360 * math.acos(bearing_z) / (2 * math.pi)
            if vec_x < 0:
                bearing *= -1
            return bearing
    # end method bearing_for_vec

    # pitch up is positive, pitch down is negative
    def calculate_pitch(self, forward):
        forward_y = forward[1]
        return 360 * math.asin(forward_y) / (2 * math.pi)
    # end method calculate_pitch

    # calculate grade (angle of the slope robot is on, regardless of robot's orientation)
    def calculate_grade(self, up):
        up_y = up[1]
        # this is sometimes (within machine precision?) outside of [-1, 1]?
        up_y = max(min(up_y, 1), -1)
        return 360 * math.acos(abs(up_y)) / (2 * math.pi)
    # end method calculate_grade

    # roll to the right is positive, roll to the left is negative
    def calculate_roll(self, right):
        right_y = right[1]
        return -360 * math.asin(right_y) / (2 * math.pi)
    # end method calculate_roll

    # return signed speed
    #
    # note: we calculate speed along the 'forward' direction (which is actually the
    # average of the last and current forward directions)
    def calculate_speed(self, last_sensor_info, current_sensor_info):
        time_delta = current_sensor_info['time'] - last_sensor_info['time']

        # get forward vector
        fwd_last = last_sensor_info['forward']
        fwd_current = current_sensor_info['forward']
        fwd = (fwd_last + fwd_current)  # average (but no need to divide by 2, will normalize below)
        fwd_norm = np.linalg.norm(fwd)
        if fwd_norm < 1e-10:
            return 0  # in the extremely unlikely event that the average fwd vector is essentially zero
        fwd = fwd / fwd_norm  # normalize

        # get position delta
        pos_last = last_sensor_info['position']
        pos_current = current_sensor_info['position']
        pos_delta = pos_current - pos_last

        # get total distance moved
        total_distance = np.linalg.norm(pos_delta)
        # get component of pos_delta in the direction of the unit vector fwd
        # (note: forward distance can be negative)
        fwd_distance = np.dot(pos_delta, fwd)
        # calculate speed
        speed = fwd_distance / time_delta
        # robot is skidding if magnitude of fwd_distance is less than 90% of total distance
        skidding = abs(fwd_distance) < 0.9 * total_distance

        return speed, skidding
    # end method calculate_speed

    # get latest sensors from the robot, calculate derived values, and store in sensor history
    # NOTE: this runs in its own thread (see constructor)
    def update_sensors(self):
        while self.running:
            cycle_start_time = time()

            num_in_history = len(self.sensor_history)

            # get current sensors from robot
            new_sensor_info = dict()

            # 2FIX: IS THERE (OR CAN THERE BE) A WAY TO GET A COHERENT SNAPSHOT OF ALL SENSORS AT A SINGLE INSTANT?

            self.sensor_lock.acquire()

            if (num_in_history == 0) or (self.bot.info.timestamp != self.sensor_history[-1]['raw_timestamp']):
                self.calculate_and_store_sensor_values()
            #else:
            #    print('NO NEW SENSOR INFO')
            # end if adding good new sensors

            self.sensor_lock.release()

            # update sensor staleness
            time_since_last_sensor = (cycle_start_time - self.start_time) - self.sensor_history[-1]['time']
            self.sensors_are_stale = time_since_last_sensor > self.max_sensor_staleness
            #if self.sensors_are_stale:
            #    print('######### SENSORS ARE STALE... ({:0.1f} milliseconds) #########'.format(1000 * time_since_last_sensor))

            # TEMP: REPORT CURRENT UPDATE FREQUENCY
            if False:  # len(self.sensor_history) > 1:
                #print(len(self.sensor_history))
                #print(self.sensor_history[-1].keys())
                time_deltas = [self.sensor_history[i + 1]['time'] - self.sensor_history[i]['time'] \
                               for i in range(len(self.sensor_history) - 1)]
                time_delta = np.mean(time_deltas)
                freq = 1 / time_delta
                #print('update frequency is {:0.1f} times/second'.format(freq))

            sleep_until_end_of_cycle(cycle_start_time, self.cycle_duration)
        # end while running
    # end method update_sensors

    # calculate all sensor-derived values and store them in the sensor history
    def calculate_and_store_sensor_values(self):
        new_sensor_info = dict()

        ### deal with sensor timestamp and staleness
        timestamp = self.bot.info.timestamp  # get robot-info timestamp (milliseconds since epoch)
        new_sensor_info['raw_timestamp'] = timestamp  # record raw timestamp
        timestamp = timestamp / 1000  # convert to seconds
        timestamp -= self.start_time  # make it since the start of the game, just 'cause
        new_sensor_info['time'] = timestamp
        #print('newest sensor timestamp is {:.2f}'.format(timestamp))

        if len(self.sensor_history) < 1:
            time_delta = None
        else:
            time_delta = new_sensor_info['time'] - self.sensor_history[-1]['time']

        # flip coordinate system if it's upside down
        if self.bot.info.coordinates_are_inverted != self.bot.info.gyroscope.is_upside_down:
            self.bot.flip_coordinates()
        if self.bot.info.coordinates_are_inverted != self.bot.info.gyroscope.is_upside_down:
            print('WARNING: I THOUGHT FLIPPING COORDINATES WAS SUPPOSED TO BE IMMEDIATE?!')

        ### record various values
        new_sensor_info['position'] = xyz_vec(self.bot.info.gps.position)
        new_sensor_info['forward'] = xyz_vec(self.bot.info.gyroscope.forward)
        new_sensor_info['right'] = xyz_vec(self.bot.info.gyroscope.right)
        new_sensor_info['up'] = xyz_vec(self.bot.info.gyroscope.up)
        new_sensor_info['bearing'] = self.bearing_for_vec(new_sensor_info['forward'])
        new_sensor_info['pitch'] = self.calculate_pitch(new_sensor_info['forward'])
        new_sensor_info['grade'] = self.calculate_grade(new_sensor_info['up'])
        new_sensor_info['roll'] = self.calculate_roll(new_sensor_info['right'])
        new_sensor_info['height_above_ground'] = self.bot.info.altimeter.altitude
        new_sensor_info['upside_down'] = self.bot.info.gyroscope.is_upside_down
        # other players' locations
        new_sensor_info['pings'] = [xyz_vec(p) for p in self.bot.info.radar.pings]
        # for playing tag
        new_sensor_info['is_it'] = self.bot.info.isIt
        # location of the player who's currently 'it' in tag
        new_sensor_info['it_ping'] = xyz_vec(self.bot.info.radar.it_ping)

        new_sensor_info['torque'] = self._torque  # this doesn't actually come from the robot sensors (whatevs)

        # special case (lidar doesn't always have exactly 360 values, grrr)
        new_sensor_info['lidar_values'] = np.array(self.bot.info.lidar.distance_matrix)
        '''
        # FIXED?
        if new_sensor_info['lidar_values'].size > 360:
            # note: this only makes sense if the last values are the bad ones, not the first values
            new_sensor_info['lidar_values'] = new_sensor_info['lidar_values'][:360]
        elif new_sensor_info['lidar_values'].size < 360:
            print('##### WARNING: LIDAR HAS FEWER THAN 360 ENTRIES #####')
            # raise ValueError('lidar should never have fewer than 360 entries')
        '''

        ### calculate and record various rates of change

        # calculate signed forward speed and skidding (boolean)
        if len(self.sensor_history) < 1:
            speed = None
            skidding = False
        else:
            speed, skidding = self.calculate_speed(self.sensor_history[-1], new_sensor_info)
        new_sensor_info['speed'] = speed
        new_sensor_info['skidding'] = skidding
        '''
        if skidding:
            print('SKIDDING...')
        else:
            print('not skidding...')
        '''

        # calculate acceleration
        if len(self.sensor_history) < 2:
            acceleration = None
        else:
            last_speed = self.sensor_history[-1]['speed']
            current_speed = new_sensor_info['speed']
            acceleration = (current_speed - last_speed) / time_delta
        new_sensor_info['acceleration'] = acceleration

        # calculate yaw rate
        # note: we can't know if the change is the long way or the short way around the circle
        # we'll just assume it's always the short way
        if len(self.sensor_history) < 1:
            yaw_rate = None
        else:
            bearing_last = self.sensor_history[-1]['bearing']
            bearing_current = new_sensor_info['bearing']
            if bearing_last is None or bearing_current is None:
                yaw_rate = None
            else:
                yaw_delta = self.calculate_yaw_delta(bearing_last, bearing_current)
                yaw_rate = yaw_delta / time_delta
        new_sensor_info['yaw_rate'] = yaw_rate

        # calculate yaw acceleration
        if len(self.sensor_history) < 2:
            yaw_acceleration = None
        else:
            last_yaw_rate = self.sensor_history[-1]['yaw_rate']
            current_yaw_rate = new_sensor_info['yaw_rate']
            yaw_acceleration = (current_yaw_rate - last_yaw_rate) / time_delta
        new_sensor_info['yaw_acceleration'] = yaw_acceleration

        # calculate pitch rate
        if len(self.sensor_history) < 1:
            pitch_rate = None
        else:
            pitch_last = self.sensor_history[-1]['pitch']
            pitch_current = new_sensor_info['pitch']
            pitch_delta = pitch_current - pitch_last
            pitch_rate = pitch_delta / time_delta
        new_sensor_info['pitch_rate'] = pitch_rate

        # calculate grade rate
        if len(self.sensor_history) < 1:
            grade_rate = None
        else:
            grade_last = self.sensor_history[-1]['grade']
            grade_current = new_sensor_info['grade']
            grade_delta = grade_current - grade_last
            grade_rate = grade_delta / time_delta
        new_sensor_info['grade_rate'] = grade_rate

        # calculate roll rate
        if len(self.sensor_history) < 1:
            roll_rate = None
        else:
            roll_last = self.sensor_history[-1]['roll']
            roll_current = new_sensor_info['roll']
            roll_delta = roll_current - roll_last
            roll_rate = roll_delta / time_delta
        new_sensor_info['roll_rate'] = roll_rate

        ### add new sensor values to the history, on the right
        self.sensor_history.append(new_sensor_info)

        ### if needed, remove the oldest sensors, on the left
        if len(self.sensor_history) > self.keep_num_sensors:
            self.sensor_history = self.sensor_history[-self.keep_num_sensors:]


        ### calculate and record various smoothed rates of change
        # use fewer samples if that's all we have
        num_smoothing_samples = min(self.num_smoothing_samples, len(self.sensor_history))
        # drop any samples that are too old, according to a simple heuristic
        # example: for 5 samples and a 50-millisecond cycle time, max time delta is 0.5 seconds
        max_time_delta = self.cycle_duration * num_smoothing_samples * 2
        t0 = self.sensor_history[-1]['time']
        recent_idxs = [i for i in range(-num_smoothing_samples, 0) if \
                       (t0 - self.sensor_history[i]['time']) < max_time_delta]
        start_idx = recent_idxs[0]  #  note that start_idx is negative

        self.sensor_history[-1]['wobble'] = self.calculate_wobble(start_idx)

        self.sensor_history[-1]['smoothed_position'] = self.calculate_smoothed_value('position', start_idx)
        self.sensor_history[-1]['smoothed_speed'] = self.calculate_smoothed_value('speed', start_idx)
        self.sensor_history[-1]['smoothed_acceleration'] = self.calculate_smoothed_value('acceleration', start_idx)
        self.sensor_history[-1]['smoothed_pitch'] = self.calculate_smoothed_value('pitch', start_idx)
        self.sensor_history[-1]['smoothed_grade'] = self.calculate_smoothed_value('grade', start_idx)
        self.sensor_history[-1]['smoothed_roll'] = self.calculate_smoothed_value('roll', start_idx)
        self.sensor_history[-1]['smoothed_yaw_rate'] = self.calculate_smoothed_value('yaw_rate', start_idx)
        self.sensor_history[-1]['smoothed_yaw_acceleration'] = self.calculate_smoothed_value('yaw_acceleration', start_idx)
        self.sensor_history[-1]['smoothed_pitch_rate'] = self.calculate_smoothed_value('pitch_rate', start_idx)
        self.sensor_history[-1]['smoothed_roll_rate'] = self.calculate_smoothed_value('roll_rate', start_idx)
        self.sensor_history[-1]['smoothed_grade_rate'] = self.calculate_smoothed_value('grade_rate', start_idx)
        self.sensor_history[-1]['smoothed_lidar_values'] = self.calculate_smoothed_value('lidar_values', start_idx)
        self.sensor_history[-1]['smoothed_height_above_ground'] = self.calculate_smoothed_value('height_above_ground', start_idx)
        self.sensor_history[-1]['smoothed_torque'] = self.calculate_smoothed_value('torque', start_idx)

        # special case: if the bearing values to be smoothed cross the -180/180 discontinuity (for bearing 'south'),
        # the mean value can come out to be right around zero.  oops.  the corrected logic is a bit ugly (oh, well)
        bearings = [self.sensor_history[i]['bearing'] for i in range(start_idx, 0)]
        bearings = [b for b in bearings if b is not None]
        if len(bearings):
            # this is a hacky way to catch the case, but whatever
            if (np.sign(min(bearings)) != np.sign(max(bearings))) and min([abs(b) for b in bearings]) > 90:
                convert = lambda x: (np.sign(x) * 180) - x
                bearings = [convert(b) for b in bearings]  # convert so the values are around zero
                smoothed_bearing = sum(bearings) / len(bearings)
                smoothed_bearing = convert(smoothed_bearing)  # convert back
            else:
                # do the simple thing
                smoothed_bearing = sum(bearings) / len(bearings)
        else:
            smoothed_bearing = None
        self.sensor_history[-1]['smoothed_bearing'] = smoothed_bearing
    # end method calculate_and_store_sensor_values

    def calculate_smoothed_value(self, name, start_idx):
        values = [self.sensor_history[i][name] for i in range(start_idx, 0)]
        values = [v for v in values if v is not None]
        if len(values):
            # note: this syntax works for both scalars and numpy arrays
            smoothed_value = sum(values) / len(values)
        else:
            smoothed_value = None
        return smoothed_value
    # end method calculate_smoothed_value

    # return signed yaw delta
    # note: we can't know if the change is the long way or the short way around the circle
    # we assume here that it's always the short way
    def calculate_yaw_delta(self, yaw1, yaw2):
        yaw_delta = yaw2 - yaw1
        if abs(yaw_delta) > 180:
            # go the short way around instead (always flipping the sign)
            yaw_delta = (abs(yaw_delta) - 360) * np.sign(yaw_delta)
        return yaw_delta
    # end method calculate_yaw_delta

    def calculate_wobble(self, start_idx):
        # get 'up' vectors
        up_vecs = [self.sensor_history[i]['up'] for i in range(start_idx, 0)]

        # get average 'up' vector (add and renormalize)
        mean_up = sum(up_vecs)
        mean_up = mean_up / np.linalg.norm(mean_up)

        # for each up vector, get its angle with the average up vector
        # (max/min is required because dot products occasionally fall within machine precision
        # outside of [-1,1])
        dot_products = [max(min(np.dot(up, mean_up), 1), -1) for up in up_vecs]
        angles = [math.acos(dp) * 360 / (2 * math.pi) for dp in dot_products]
        wobble = np.mean(angles)

        return wobble
    # end method calculate_wobble

    def set_bot_steering(self, angle):
        self.bot.set_tire_steering("FrontLeft", angle)
        self.bot.set_tire_steering("FrontRight", angle)
        self.bot.set_tire_steering("BackLeft", -angle)
        self.bot.set_tire_steering("BackRight", -angle)
    # end function set_bot_steering

    def set_bot_torque(self, torque):
        # all-wheel drive
        self.bot.set_tire_torque("FrontLeft", torque)
        self.bot.set_tire_torque("FrontRight", torque)
        self.bot.set_tire_torque("BackLeft", torque)
        self.bot.set_tire_torque("BackRight", torque)
    # end function set_bot_torque

    # rotate in place
    # positive torque rotates right (clockwise), negative torque rotates left (counterclockwise)
    def rotate(self, torque):
        # set torque
        torque = np.sign(torque) * min(abs(torque), self.max_abs_torque)  # apply absolute limit
        self._torque = torque

        steering_angle = 45  # this may not be the optimum angle for rotation, but it seems close enough

        if self.upside_down:
            # flip_coordinates switches everything except the names of the wheels
            flip = -1
        else:
            flip = 1

        self.bot.set_tire_steering("FrontLeft", steering_angle * flip)
        self.bot.set_tire_torque("FrontLeft", torque * flip)

        self.bot.set_tire_steering("FrontRight", -steering_angle * flip)
        self.bot.set_tire_torque("FrontRight", -torque * flip)

        self.bot.set_tire_steering("BackLeft", -steering_angle * flip)
        self.bot.set_tire_torque("BackLeft", torque * flip)

        self.bot.set_tire_steering("BackRight", steering_angle * flip)
        self.bot.set_tire_torque("BackRight", -torque * flip)
    # end method rotate

    # put wheels into the rotating configuration to minimize rolling
    def park(self):
        self.rotate(0)

    # rotate in place to face the given target bearing
    # return a boolean -- true if robot is currently within bearing_tolerance of tgt_bearing
    def rotate_to_bearing(self, tgt_bearing, bearing_tolerance = 10, yaw_rate_tolerance=10):
        if tgt_bearing < -180 or tgt_bearing > 180:
            raise ValueError('target bearing must be in [-180,180]')

        bearing = self.smoothed_bearing
        yaw_rate = self.smoothed_yaw_rate

        bearing_delta = self.calculate_yaw_delta(bearing, tgt_bearing)
        tgt_yaw_rate = bearing_delta
        yaw_rate_delta = tgt_yaw_rate - yaw_rate

        # 2FIX: there's a bad feedback loop where the robot starts spinning like a top.  how
        # does that happen?  will this regulation here fix it?
        # set a maximum yaw_rate_delta
        yaw_rate_delta = np.sign(yaw_rate_delta) * min(abs(yaw_rate_delta), 200)

        # 2FIX: IS IT SCREWY TO USE BEARING_TOLERANCE FOR THIS PURPOSE HERE?
        if abs(yaw_rate) < bearing_tolerance and abs(yaw_rate_delta) > bearing_tolerance:
            # if turning slowly but want to be turning fast, keep increasing torque until robot moves
            torque_delta = np.sign(yaw_rate_delta) * 50
            torque = self.torque + torque_delta
            print('rotating -- increasing torque to {} (yrd {}, yr {}, tyr{})'.format(torque, yaw_rate_delta, yaw_rate, tgt_yaw_rate))
        else:
            torque = 3 * yaw_rate_delta
            print('rotating -- setting torque to {} (yrd {}, yr {}, tyr{})'.format(torque, yaw_rate_delta, yaw_rate, tgt_yaw_rate))

        arrived = (abs(bearing_delta) <= bearing_tolerance) and (abs(yaw_rate) < yaw_rate_tolerance)
        if arrived:
            self.torque = 0
        else:
            self.rotate(torque)

        return arrived
    # end method rotate_to_bearing

    def rotate_to_face_downhill(self, bearing_tolerance = 20, yaw_rate_tolerance=20):
        downhill_bearing = self.bearing_for_vec(self.up)
        if downhill_bearing is None:
            return False  # robot is level (not on a hill)

        arrived = self.rotate_to_bearing(downhill_bearing, bearing_tolerance, yaw_rate_tolerance)

        return arrived
    # end method rotate_to_face_downhill

    def rotate_to_face_uphill(self, bearing_tolerance = 10, yaw_rate_tolerance=10):
        uphill_bearing = self.bearing_for_vec(-self.up)
        if uphill_bearing is None:
            return False  # robot is level (not on a hill)

        arrived = self.rotate_to_bearing(uphill_bearing, bearing_tolerance, yaw_rate_tolerance)

        return arrived
    # end method rotate_to_face_uphill

    def rotate_n_degrees(self, delta):
        tgt_bearing = add_bearings(self.smoothed_bearing, delta)
        self.rotate_to_bearing(tgt_bearing)
    # end method rotate_n_degrees

    # set a target speed
    # note: heuristics are designed to go as fast as possible without flipping over
    def get_tgt_speed(self):
        if self.smoothed_height_above_ground > 1:
            # 2FIX: WILL THE ROBOT EVERY GET STUCK SOMEWHERE WHERE THE WHEELS ARE ON THE GROUND
            # BUT THE BODY OF THE ROBOT IS OVER A CREVICE (OR SOMESUCH?)
            return 0

        min_speed = 3
        max_speed = 12

        pitch = self.smoothed_pitch
        if pitch > 0:  # uphill
            pitch_tgt_speed = max_speed - (pitch / 2.5)
        else:  # downhill (pitch is negative)
            pitch_tgt_speed = max_speed + (pitch / 3.5)

        wobble_tgt_speed = max_speed - (self.wobble * 3.5)

        steering_tgt_speed = max_speed - (abs(self.steering_angle) / 1.5)

        tgt_speed = min(pitch_tgt_speed, wobble_tgt_speed, steering_tgt_speed)
        tgt_speed = max(tgt_speed, min_speed)

        # impose a separate speed limit when the current speed is low.
        # this is needed because a tight spot that requires a lot of torque to get out of
        # may be smooth, straight, and level - so the robot quickly gets going too fast once
        # it gets unstuck
        # (note: the real solution to this problem involves lidar...)
        if abs(self.smoothed_speed) < 1:
            tgt_speed = min(tgt_speed, 6)

        # if robot is running away and chaser is getting close, take the risk of bumping up the max
        # speed a little
        pos_delta = self.state['tgt_pos_delta']
        pos_delta[1] = 0  # ignore vertical distance (y) completely
        tgt_distance = np.linalg.norm(pos_delta)
        if self.is_it and tgt_distance < 30:
            # boost is 0% at 30m, ramping up to ????% as chaser gets very close
            boost = 1  # 1 + 0.02 * (30 - tgt_distance)
            tgt_speed *= boost
        if not self.is_it and tgt_distance < 100:
            # boost is 0% at 100m, ramping up to 200% as chaser gets very close
            boost = 1  # 1 + 0.01 * (100 - tgt_distance)
            tgt_speed *= boost

        return tgt_speed
    # end function get_tgt_speed

    def regulate_torque(self, tgt_speed):
        ### first, a few failure modes
        # if we're blind, ease off the gas a little
        if self.sensors_are_stale:
            #print('sensors stale -- easing off the gas...')
            self.torque *= 0.98
            return
        # if we're off the ground, ease off the gas more than a little
        height = self.smoothed_height_above_ground
        if height > 0.8:
            #print('in midair, at height: {:.1f} -- easing off the gas...'.format(height))
            self.torque *= 0.95
            return
        # avoid crazy response when the robot 'teleports' back to the starting point (on reset)
        if abs(self.smoothed_speed) > 50 or abs(self.smoothed_acceleration) > 50:
            self.torque = 0
            return

        # get sensor values
        speed = self.smoothed_speed
        acceleration = self.smoothed_acceleration
        pitch = self.smoothed_pitch

        speed_delta = tgt_speed - speed
        tgt_acceleration = speed_delta
        acceleration_delta = tgt_acceleration - acceleration

        # if acceleration delta is positive, (signed) torque needs to be bigger, and vice versa
        torque_delta = np.sign(acceleration_delta) * (abs(acceleration_delta) ** 1.7)

        # change torque faster on a steeper slope
        if pitch > 0:
            pitch_boost = 1 + (pitch / 8)
        else:
            pitch_boost = 1 - (pitch / 30)
        torque_delta *= pitch_boost

        # adjust aggressiveness and limit max step size based on current torque
        if np.sign(self.torque) == np.sign(torque_delta):
            # torque is increasing
            if abs(self.torque) < 500:
                # if torque is low, be more aggressive about increasing it
                torque_delta *= 1.2
            # apply max step size of 100
            torque_delta = np.sign(torque_delta) * min(abs(torque_delta), 150)
        else:
            # torque is decreasing
            if abs(self.torque) > 1500:
                # if torque is high, be more aggressive about decreasing it
                torque_delta *= 1.2
            # apply max step size of 200 (max step size is bigger for decreasing torque)
            torque_delta = np.sign(torque_delta) * min(abs(torque_delta), 300)

        # if speed is low and torque is low, be more aggressive about increasing it (yet again)
        if abs(speed) < 2 and abs(self.torque) < 300:
            torque_delta *= 3

        self.torque += torque_delta
    # end function regulate_torque

    # steer the robot toward its target position
    def steer_toward_target(self):
        tgt_pos_delta = self.state['tgt_pos_delta']

        #print('steering toward position: ', tgt_pos)

        if self.smoothed_bearing is None:
            if DEBUG_TATTLES:
                print("bearing is None - can't steer")
            return # can't steer if we don't know which way we're facing

        delta_vec = tgt_pos_delta
        delta_vec[1] = 0  # ignore vertical distance (y) completely
        distance = np.linalg.norm(delta_vec)
        if DEBUG_TATTLES:
            print('distance to target is {:.1f}'.format(distance))

        tgt_bearing = self.bearing_for_vec(delta_vec)
        if DEBUG_TATTLES:
            print('target bearing: {:+0.1f}'.format(tgt_bearing))

        tgt_bearing = self.chasing_roll_course_correction(tgt_bearing, distance)
        if DEBUG_TATTLES:
            print('corrected target bearing: {:+0.1f}'.format(tgt_bearing))

        bearing_delta = self.calculate_yaw_delta(self.smoothed_bearing, tgt_bearing)
        if DEBUG_TATTLES:
            print('bearing delta: {:+.1f}'.format(bearing_delta))

        # note: ZRobot enforces a maximum absolute steering angle (20 degrees), so large values here
        # just mean 'hard left' or 'hard right'
        coef = 0.05 * (distance ** -0.5)
        steering_angle = coef * np.sign(bearing_delta) * (bearing_delta ** 2)
        if DEBUG_TATTLES:
            print('steering angle: {:+.1f}'.format(steering_angle))

        ### apply steering angle to robot
        self.steering_angle = steering_angle
        if DEBUG_TATTLES:
            print('bot steering angle: {:+.1f}'.format(self.steering_angle))
    # end function steer_toward_target

    # steer the robot away from its 'target' position (the chaser)
    def steer_away_from_target(self):
        tgt_pos_delta = self.state['tgt_pos_delta']

        if DEBUG_TATTLES:
            print('steering away from position: ', tgt_pos)

        if self.smoothed_bearing is None:
            if DEBUG_TATTLES:
                print("bearing is None - can't steer")
            return # can't steer if we don't know which way we're facing

        delta_vec = -tgt_pos_delta  # note the sign: run away!
        delta_vec[1] = 0  # ignore vertical distance (y) completely
        distance = np.linalg.norm(delta_vec)
        if DEBUG_TATTLES:
            print('distance to target is {:.1f}'.format(distance))

        tgt_bearing = self.bearing_for_vec(delta_vec)
        if DEBUG_TATTLES:
            print('target bearing: {:+0.1f}'.format(tgt_bearing))

        tgt_bearing = self.fleeing_roll_course_correction(tgt_bearing, distance)
        if DEBUG_TATTLES:
            print('corrected target bearing: {:+0.1f}'.format(tgt_bearing))

        bearing_delta = self.calculate_yaw_delta(self.smoothed_bearing, tgt_bearing)
        if DEBUG_TATTLES:
            print('bearing delta: {:+.1f}'.format(bearing_delta))

        # note: ZRobot enforces a maximum absolute steering angle (20 degrees), so large values here
        # just mean 'hard left' or 'hard right'
        coef = 0.05 * (distance ** -0.5)
        steering_angle = coef * np.sign(bearing_delta) * (bearing_delta ** 2)
        if DEBUG_TATTLES:
            print('steering angle: {:+.1f}'.format(steering_angle))

        ### apply steering angle to robot
        self.steering_angle = steering_angle
        if DEBUG_TATTLES:
            print('bot steering angle: {:+.1f}'.format(self.steering_angle))
    # end function steer_away_from_target

    def steer_downhill(self):
        if self.smoothed_bearing is None:
            if DEBUG_TATTLES:
                print("steering downhill -- bearing is None - can't steer")
            return # can't steer if we don't know which way we're facing

        tgt_pos = self.state['tgt_pos_delta']
        if DEBUG_TATTLES:
            print('steering downhill -- tgt position: ', tgt_pos)

        delta_vec = tgt_pos_delta  # vector *toward* the target
        delta_vec[1] = 0  # ignore vertical distance (y) completely
        distance = np.linalg.norm(delta_vec)
        if DEBUG_TATTLES:
            print('distance to target is {:.1f}'.format(distance))

        tgt_bearing = self.bearing_for_vec(delta_vec)
        if DEBUG_TATTLES:
            print('target bearing: {:+0.1f}'.format(tgt_bearing))

        downhill_bearing = self.bearing_for_vec(self.up)  # 2FIX: ENCAPSULATE THIS IN A FUNCTION/METHOD
        if downhill_bearing is None:
            if self.is_it:
                self.steer_toward_target()
            else:
                self.steer_away_from_target()
            return

        tgt_bearing_delta = self.calculate_yaw_delta(downhill_bearing, tgt_bearing)
        if self.is_it:
            # steer toward target, with a limit of +/- 45 degrees from downhill
            deviation_from_downhill = np.sign(tgt_bearing_delta) * min(abs(tgt_bearing_delta), 45)
            final_bearing = add_bearings(downhill_bearing, deviation_from_downhill)
        else:
            # if chaser is behind robot, just steer downhill
            # otherwise, steer away from target, with a limit of +/- 45 degrees from downhill
            # deviation is 0 degrees when target is at +/- 180 degrees from downhill
            # deviation maxes out at 45 degrees
            deviation_from_downhill = (180 - abs(tgt_bearing_delta))  # positive
            deviation_from_downhill = min(deviation_from_downhill, 45)  # still positive
            deviation_from_downhill *= -np.sign(tgt_bearing_delta)  # add sign (away from chaser)
            final_bearing = add_bearings(downhill_bearing, deviation_from_downhill)

        bearing_delta = self.calculate_yaw_delta(self.smoothed_bearing, final_bearing)
        if DEBUG_TATTLES:
            print('bearing delta: {:+.1f}'.format(bearing_delta))

        # note: ZRobot enforces a maximum absolute steering angle (20 degrees), so large values here
        # just mean 'hard left' or 'hard right'
        coef = 0.05 * (distance ** -0.5)
        steering_angle = coef * np.sign(bearing_delta) * (bearing_delta ** 2)
        if DEBUG_TATTLES:
            print('steering angle: {:+.1f}'.format(steering_angle))

        ### apply steering angle to robot
        self.steering_angle = steering_angle
        if DEBUG_TATTLES:
            print('bot steering angle: {:+.1f}'.format(self.steering_angle))

        ##### OLD SIMPLE WAY: GO STRAIGHT DOWNHILL
        ########### self.steering_angle = 0.2 * self.smoothed_roll  ## steer toward roll to go downhill

    # end function steer_downhill

    # allow some leeway to go around the steepest parts of the terrain
    # simple logic: if roll is to the right, turn right, and vice versa
    def chasing_roll_course_correction(self, tgt_bearing, distance):
        # maximum allowed course correction goes down as the target gets closer
        # always in the range [0, 30]
        # +/- 10 degrees at 200 m, +/- 5 degrees at 100 m, reaches 0 at 20 m
        max_course_correction = 0.05 * distance - 1
        max_course_correction = max(min(max_course_correction, 30), 0)

        ### apply course correction to target bearing
        correction = self.state['roll_course_correction']
        # apply logic
        if abs(self.smoothed_roll) > 5 or self.smoothed_pitch > 10:
            roll_sign = np.sign(self.smoothed_roll)
            # turn right if rolling right, turn left if rolling left
            correction += roll_sign * 0.3
            # apply max limit
            correction = np.sign(correction) * min(abs(correction), max_course_correction)
        else:
            # move course correction back toward zero
            correction -= np.sign(correction) * 0.9
        # save new value in state
        self.state['roll_course_correction'] = correction

        if DEBUG_TATTLES:
            print('applying course correction {:+0.1f}'.format(correction))

        # the math of adding bearings is just a little messy
        tgt_bearing = add_bearings(tgt_bearing, correction)

        return tgt_bearing
    # end function chasing_roll_course_correction

    # allow some leeway to go around the steepest parts of the terrain
    # simple logic: if roll is to the right, turn right, and vice versa
    def fleeing_roll_course_correction(self, tgt_bearing, distance):
        # when fleeing, allow more leeway as the robot goes up steeper hills
        if self.smoothed_pitch < 10:
            max_course_correction = 45
        elif self.smoothed_pitch < 15:
            max_course_correction = 90
        else:
            max_course_correction = 120

        ### apply course correction to target bearing
        correction = self.state['roll_course_correction']
        correction_sign = np.sign(correction)
        # apply logic
        if abs(self.smoothed_roll) > 5 or self.smoothed_pitch > 10:
            roll_sign = np.sign(self.smoothed_roll)
            # turn right if rolling right, turn left if rolling left
            correction += roll_sign * 0.3

            at_max = abs(correction) >= max_course_correction
            correction_above_90 = max_course_correction > 90
            signs_match = correction_sign == roll_sign
            if at_max and correction_above_90 and signs_match:
                # special case: if we're:
                # a) at max course correction, and
                # b) max course correction is > 90, and
                # c) sign of correction == sign of roll (so we're pushing up against correction limit),
                # then flip the correction to do a 180-degree turn instead
                correction = correction_sign * max_course_correction - 180 * correction_sign
            else:
                # apply max limit
                correction = np.sign(correction) * min(abs(correction), max_course_correction)
        else:
            # move course correction back toward zero
            correction -= np.sign(correction) * 0.9
        # save new value in state
        self.state['roll_course_correction'] = correction

        if DEBUG_TATTLES:
            print('applying course correction {:+0.1f}'.format(correction))

        # the mat of adding bearings is just a little messy
        tgt_bearing = add_bearings(tgt_bearing, correction)

        return tgt_bearing
    # end function roll_course_correction

    # stuck is set to false in get_unstuck
    def is_stuck(self):
        t = time()
        if abs(self.smoothed_speed) > 0.1:
            self.state['last_moved_time'] = t
        if t - self.state['last_moved_time'] > self.state['stuck_timeout']:
            self.state['stuck'] = True
            print('STUCK')
        return self.state['stuck']
    # end function is_stuck

    # to get unstuck, rotate to face downhill and move downhill
    def get_unstuck(self):
        if not self.state['moving_downhill'] and not self.facing_downhill:
            self.rotate_to_face_downhill()
        else:
            t = time()
            if not self.state['moving_downhill']:
                self.state['moving_downhill'] = True
                self.state['moving_downhill_time'] = t
                print('MOVING_DOWNHILL')
            self.steer_downhill()
            tgt_speed = self.get_tgt_speed()
            self.regulate_torque(tgt_speed)
            if t - self.state['moving_downhill_time'] > self.state['moving_downhill_timeout']:
                self.state['moving_downhill'] = False
                self.state['stuck'] = False
                print('UNSTUCK')
    # end function get_unstuck

    # check if robot is on a hill that's too steep
    # note: setting too_steep to false happens in head_downhill
    def uphill_too_steep(self):
        t = time()
        if self.smoothed_grade < self.state['uphill_grade_threshold']:
            self.state['last_level_time'] = t
        if t - self.state['last_level_time'] > self.state['uphill_timeout']:
            self.state['too_steep'] = True
            print('TOO STEEP')
        return self.state['too_steep']
    # end function uphill_too_steep

    def head_downhill(self):
        if not self.state['moving_downhill'] and not self.facing_downhill:
            self.rotate_to_face_downhill()
        else:
            t = time()
            if not self.state['moving_downhill']:
                self.state['moving_downhill'] = True
                print('MOVING_DOWNHILL')
            self.steer_downhill()
            tgt_speed = self.get_tgt_speed()
            self.regulate_torque(tgt_speed)
            if self.smoothed_grade > self.state['downhill_grade_threshold']:
                self.state['last_steep_time'] = t
            if t - self.state['last_steep_time'] > self.state['moving_downhill_timeout']:
                self.state['moving_downhill'] = False
                self.state['too_steep'] = False
                print('NOT TOO STEEP')
    # end function head_downhill

    def chase(self):
        # when chasing, do the simple thing: if in a bad state, fix the bad state (ignoring
        # chasing until the bad state is fixed)
        if self.is_stuck():
            self.get_unstuck()
        elif self.uphill_too_steep():
            self.head_downhill()
        else:
            self.steer_toward_target()
            tgt_speed = self.get_tgt_speed()
            self.regulate_torque(tgt_speed)
    # end function chase

    def flee(self):
        # 2FIX: INCORPORATE FLEEING AND GOING DOWNHILL TOGETHER?
        if self.is_stuck():
            self.get_unstuck()
        elif self.uphill_too_steep():
            self.head_downhill()
        else:
            self.steer_away_from_target()
            tgt_speed = self.get_tgt_speed()
            self.regulate_torque(tgt_speed)
    # end function flee

# end class ZRobot

if __name__ == '__main__':
    bot = ZRobot()
# end main
