from awconnection import RobotConnection
import time, math
import scipy as sp
import numpy as np
import datetime
import tkinter as tk
import time, math

robot = RobotConnection()
robot.connect()

half_map_width = 50
half_map_height = 50

savetime = int(round(time.time()*1000))

base_speed = 2  # Use 2.2 for tag, 1.8 for Fight, 1.2 for Explore

any_fails = False

pos = robot.info.gps.position
last_gps = np.array([pos.x,pos.y,pos.z])
start_maps_over = False

last_max_speed = 0

Flipped = False
Pirouetting = False

then = int(round(time.time()*1000))

# up = bot.info.gyroscope.up
# print(up.x)
# print(up.y)
# print(up.z)
# bot.disconnect()

# THINGS TO DO

# Fix charge enemy and flee enemy so that they take a robot.dict instead of an index

# Get Lidar helping in bias_off_roll_and_clear_space

# Get Lidar helping to see if it is bumpy

# Get Lidar helping to slow down if things are dangerous

# Start a learning systetm to optimize characteristics to minimize flipping

# set up so flipping isn't a problem for pirouetting

# SAVE!!

current_torque = 100
goal_direction = np.random.random()*2*3.14159-3.14159 # in radians

steer_radians = 0 # in radians
max_steer = .32  # in radians
debug = False

last_speed = 0
speed_burst = 1.4
max_torque = 1600

then = int(round(time.time()*1000))
last_slope = 0

IT = robot.info.isIt

last_alt = [.4,.4,.4,.4,.4]
avg_alt = .41

robot_gps = robot.info.gps.position

initial_gps = np.array([robot_gps.x,robot_gps.y,robot_gps.z])
print("Initial GPS = ",initial_gps)

if debug:
    print("Initial GPS = ",initial_gps)
last_gps = initial_gps
if debug:
    print("Last GPS =",last_gps)

# ------------------------------------------
def setup_maps(new_world):

    global elevation_high_map
    global avg_speed_map
    global num_fails_map
    global num_visits_map

    if new_world:
    
       elevation_high_map = np.array([[0 for i in range(half_map_width*2)] for j in range(half_map_height*2)])
       avg_speed_map = np.array([[0 for i in range(half_map_width*2)] for j in range(half_map_height*2)])
       num_fails_map = np.array([[0 for i in range(half_map_width*2)] for j in range(half_map_height*2)])
       num_visits_map = np.array([[0 for i in range(half_map_width*2)] for j in range(half_map_height*2)])

       np.savetxt("elevation_high_map.txt",elevation_high_map)
       np.savetxt("avg_speed_map.txt",avg_speed_map)
       np.savetxt("num_fails_map.txt",num_fails_map)
       np.savetxt("num_visits_map.txt",num_visits_map)

    else:

       elevation_high_map = np.loadtxt("elevation_high_map.txt")
       avg_speed_map = np.loadtxt("avg_speed_map.txt")
       num_fails_map = np.loadtxt("num_fails_map.txt")
       num_visits_map = np.loadtxt("num_visits_map.txt")   

# ------------------------------------------
def check_radians(bearing):

    if bearing < -math.pi or bearing > math.pi:
       print(" ***   ###   AHHH!!!  Radians Broken   ###   ***   ")


#-------------------------------------
def behind_me(enemy_bearing):

    return enemy_bearing < -math.pi/2 or enemy_bearing > math.pi/2


#-------------------------------------
def head_nearly_directly_away(enemy_bearing):

    if enemy_bearing <= 0:
        goal_direction = enemy_bearing + math.pi*(1-np.random.random()*0.03)
    else:
        goal_direction = enemy_bearing - math.pi*(1-np.random.random()*0.03)

    check_radians(goal_direction)
    return goal_direction


#-------------------------------------
def on_a_serious_up_slope():

    global last_slope
    
    robot_vector = get_bearing_vec(robot.info.gyroscope.forward)

    if last_slope > 0.3 and robot_vector[1] > last_slope:
        last_slope = robot_vector[1]
        return True
    else: 
        last_slope = robot_vector[1]
        return False


#-------------------------------------
def slope():
    
    robot_vector = get_bearing_vec(robot.info.gyroscope.forward)

    return robot_vector[1]
        
    
#-------------------------------------
def rad_safe_add(goal_dir, change):

    goal_dir = goal_dir + change
    if goal_dir > math.pi:
        goal_dir -= 2*math.pi
    elif goal_dir < -math.pi:
        goal_dir += 2*math.pi

    check_radians(goal_dir)
    return goal_dir

#-------------------------------------
def turn_on_slope_avoid_enemy(enemy_bearing):

    global goal_direction

    if enemy_bearing <= 0:
        goal_direction = rad_safe_add(goal_direction,math.pi / 4)
    else:
        goal_direction = rad_safe_add(goal_direction,-math.pi / 4)

    check_radians(goal_direction)
    return goal_direction
    
#-------------------------------------
def flee_enemy(enemy):

    global goal_direction
        
    enemy_bearing = get_enemy_bearing(enemy)

    if on_a_serious_up_slope(): # and enemy_close(# parametrize how close you care about here and send in the enemy, not index
       goal_direction = turn_on_slope_avoid_enemy(enemy_bearing)
    else:
       if behind_me(enemy_bearing):
          goal_direction = goal_direction
       else:
          goal_direction = head_nearly_directly_away(enemy_bearing)

    check_radians(goal_direction)
    return goal_direction

#-------------------------------------
def get_dist_to_foe(enemy):

    enemy_arr = np.array([enemy.x,enemy.y,enemy.z])
    return vec_dist(enemy_arr,[0,0,0])

#-------------------------------------
def get_enemy_bearing(enemy):

    altitudeless_bearing_vector = np.array([enemy.x,0,enemy.z])
    size = np.linalg.norm(altitudeless_bearing_vector)
    altitudeless_bearing_vector /= size

    enemy_bearing = math.acos(altitudeless_bearing_vector[2])
    if enemy.x < 0:
         enemy_bearing = -enemy_bearing

    check_radians(enemy_bearing)
    return enemy_bearing


#-------------------------------------
def charge_enemy(enemy):
    global goal_direction

    enemy_bearing = get_enemy_bearing(enemy)    
         
    goal_direction = enemy_bearing

    return goal_direction


#-------------------------------------
def set_torque(level):
    
    robot.set_tire_torque("BackLeft", level)
    robot.set_tire_torque("BackRight", level)
    robot.set_tire_torque("FrontLeft", level)
    robot.set_tire_torque("FrontRight", level)

#-------------------------------------
def steer(amount):
    
    global steer_radians
    global max_steer

    steer_radians += amount
    if steer_radians > max_steer:
      steer_radians = max_steer
    elif steer_radians < -max_steer:
       steer_radians = -max_steer

    if abs(steer_radians) < 0.002:
       steer_radians =  0
   
    robot.set_tire_steering("FrontLeft", steer_radians*57.3)   # convert to angles
    robot.set_tire_steering("FrontRight", steer_radians*57.3)  # convert to angles
    robot.set_tire_steering("BackLeft", 0)   # convert to angles
    robot.set_tire_steering("BackRight", 0)  # convert to angles


#-------------------------------------
def might_print(percent_chance,string,num):
    
   if np.random.random() < (percent_chance/100):
       print(string,num)
       return True
   else:
       return False

#-------------------------------------   
def jump():
   
    robot.set_tire_torque("BackLeft", 0)
    robot.set_tire_torque("BackRight", 0)
    robot.set_tire_torque("FrontLeft", -max_torque*10)
    robot.set_tire_torque("FrontRight", -max_torque*10)
    print("jumping!!")


#-------------------------------------   
def enemy_very_close(index,cur_speed):

    enemies = robot.info.radar.pings 

    if len(enemies) > index:
       enemy_dist = get_dist_to_foe(enemies[index])
       if enemy_dist < 2*cur_speed:   # This assume cur_speed is distance / second, so within a few seconds of touching robot
          return True
        
    return False


#-------------------------------------   
def roll():

    global robot

    robot_vector = get_bearing_vec(robot.info.gyroscope.right)
    if debug:
       print(" What I get back is ",robot_vector)

    roll = math.asin(robot_vector[1])
 
    return roll

#-------------------------------------   
def keep_speed(cur_speed,cur_bearing,target_bearing,margin):
    
    global goal_direction
    global current_torque
    global last_speed
    global steer_radians
    global last_max_speed
    global any_fails

    max_speed = base_speed

    if check_not_bumpy():
       max_speed *= 1.6
    else:
       might_print(2,"Bumpy!  ",-1)

    if cur_bearing - target_bearing < margin or cur_bearing - target_bearing > -margin:
       if abs(steer_radians) < .18:
          max_speed *=2

    cur_slope = slope()
    if abs(cur_slope) < .20:
       max_speed *= 1.6
    else:
       might_print(2,"Slope high = ",cur_slope)

    cur_roll = roll()
    if abs(cur_roll) < .20:
       max_speed *= 1.6
    else:
       might_print(2,"Roll high = ",cur_roll)

    if enemy_very_close(0,cur_speed):
      max_speed *= speed_burst

    if max_speed > last_max_speed:
       max_speed = (max_speed + last_max_speed)/2
    else:
       max_speed = (2*max_speed + last_max_speed)/3
        
    might_print(2,"Max Speed = ",max_speed)
    last_max_speed = max_speed

    if abs(slope())>0.65:

       print("Giving Up and Going Back!")
       print(current_torque, max_torque)
       print(cur_speed, max_speed, slope())
       any_fails = True
       goal_direction = rad_safe_add(goal_direction,0.1)
       current_torque = -max_torque/2

    elif cur_speed < max_speed:

       if cur_speed + (cur_speed-last_speed)*8 < max_speed:
          current_torque = min(current_torque + max_torque*0.2,max_torque)
       else:
          current_torque = min(current_torque + max_torque*0.05,max_torque)
    else:  
       current_torque = -max_torque/10

    set_torque(current_torque)
    
    last_speed = cur_speed


#-------------------------------------  
def vec_dist(vec1, vec2):

    return (((vec1[0]-vec2[0])**2)+((vec1[1]-vec2[1])**2)+((vec1[2]-vec2[2])**2))**0.5


#-------------------------------------  
def steer_control(current_speed,cur_bearing,target_bearing,margin):

    global  steer_radians
    global  max_steer

    if abs(cur_bearing - target_bearing) > math.pi*1.01 : # i.e. if it is shorter to go the other way
        if target_bearing < 0:
           target_bearing += 2*math.pi  # Not rad safe right now but it works 
        else:
           target_bearing -= 2*math.pi  # Not rad safe right now but it works 

    if current_speed > 9:
       max_steer_now = max_steer/3
    elif current_speed > 6.5:
       max_steer_now = max_steer/2
    else:
       max_steer_now = max_steer
           
    if cur_bearing - target_bearing > margin:  
        steer(-steer_radians - max_steer_now)

    elif cur_bearing - target_bearing < -margin:
        steer(-steer_radians + max_steer_now)

    elif cur_bearing - target_bearing > margin/2 and steer_radians>0:
        steer(-steer_radians - max_steer_now*.5)

    elif cur_bearing - target_bearing < -margin/2 and steer_radians<0:
        steer(-steer_radians + max_steer_now*.5)

    else:
        steer(-steer_radians*1.2)


#-------------------------------------  
def get_hillless_bearing(robot_vec):

    vector = np.array([robot_vec[0],0,robot_vec[2]])
    size = np.linalg.norm(vector)
    vector /= size

    bearing = math.acos(vector[2])  
    if vector[0] < 0:
        bearing = -bearing

    return bearing

#-------------------------------------  
def get_bearing(robot_vec):

    vector = np.array([robot_vec[0],robot_vec[1],robot_vec[2]])
    size = np.linalg.norm(vector)
    vector /= size

    bearing = math.acos(vector[2])  
    if vector[0] < 0:
        bearing = -bearing

    return bearing

#-------------------------------------  
def get_bearing_vec(robot_list):
    
    robot_vector = np.array([robot_list.x,robot_list.y,robot_list.z])
    if debug:
       print("Inside, before I normalize it is ",robot_vector)
    
    size = np.linalg.norm(robot_vector)
    robot_vector /= size
    
    return robot_vector

#-------------------------------------
def get_front_high_lidar():

    global robot

    cur_lidar = np.array(robot.info.lidar.distance_matrix)
    cur_lidar_right = np.array(cur_lidar[:8,:6])
    cur_lidar_left = np.array(cur_lidar[:8,-6:])
    cur_lidar_front = np.concatenate((cur_lidar_left, cur_lidar_right), 1)

    return cur_lidar_front

#-------------------------------------
def get_lidar_std(lidar,row):  #lidar is 2D 

    temp_arr = np.array(lidar[row])
       
    return np.std(temp_arr)
    
#-------------------------------------
def get_lidar_average(lidar,row): 

    return sum(lidar[row]) / len(lidar[row])

#-------------------------------------
def check_not_bumpy():

   global last_alt
   global avg_alt

   alt = robot.info.altimeter.altitude

   lidar = np.array(get_front_high_lidar())

   last_alt = last_alt[1:len(last_alt)]
   last_alt.append(alt)
   
   max_alt = max(last_alt)
   min_alt = min(last_alt)
   avg_alt = sum(last_alt)/len(last_alt)

   bottom_3_rows = (get_lidar_average(lidar,5)+get_lidar_average(lidar,6)+get_lidar_average(lidar,7))/3
   rise_ahead = bottom_3_rows < 30
   dip_ahead =  bottom_3_rows > 200

   # bottom row more important
   bottom_2_row_std = (get_lidar_std(lidar,6)/get_lidar_average(lidar,6) + 9*get_lidar_std(lidar,7)/get_lidar_average(lidar,7))/10  
   
   bumpy_std = bottom_2_row_std > 0.35

   return ((abs(max_alt - min_alt) / avg_alt) < 0.2) and not rise_ahead and not dip_ahead and not bumpy_std


#-------------------------------------
def outside_dir_margin(cur_bearing,target_bearing,margin):

    if cur_bearing + math.pi*1.01 < target_bearing:
       cur_bearing += 2*math.pi
    elif cur_bearing - math.pi*1.01 > target_bearing:
       cur_bearing -= 2*math.pi

    return cur_bearing-target_bearing > margin or cur_bearing-target_bearing < -margin

#-------------------------------------
def turn_wheels_for_circle(steer_amt):

    if Flipped:
       torque_amt = -steer_amt

    robot.set_tire_steering("FrontLeft", steer_amt*57.3)   # convert to angles
    robot.set_tire_steering("FrontRight", -steer_amt*57.3)  # convert to angles
    robot.set_tire_steering("BackLeft", -steer_amt*57.3)   # convert to angles
    robot.set_tire_steering("BackRight", steer_amt*57.3)  # convert to angles
    
#-------------------------------------
def run_wheels_for_pirouette(torque_amt):

    robot.set_tire_torque("BackLeft", torque_amt)
    robot.set_tire_torque("BackRight", -torque_amt)
    robot.set_tire_torque("FrontLeft", torque_amt)
    robot.set_tire_torque("FrontRight", -torque_amt)

#-------------------------------------
def pirouette(cur_bearing,target_bearing):

    wheel_angle = math.pi/4

    if abs(cur_bearing - target_bearing) > math.pi*1.01 : # i.e. if it is shorter to go the other way
        if target_bearing < 0:
           target_bearing += 2*math.pi  # Not rad safe right now but it works 
        else:
           target_bearing -= 2*math.pi

    if cur_bearing > target_bearing:
       turn_wheels_for_circle(wheel_angle)
       run_wheels_for_pirouette(-max_torque/3)
    else:
       turn_wheels_for_circle(wheel_angle)
       run_wheels_for_pirouette(max_torque/3)
    
    
#-------------------------------------
def hold_bearing_and_speed(target_bearing, margin):
    global last_gps
    global then
    global goal_direction
    global cur_bearing
    global Flipped
    global Pirouetting
    global any_fails
    
    if robot.info.gyroscope.is_upside_down and (not Flipped):
       print("Flipping!")
       Flipped = True
       any_fails = True
       robot.flip_coordinates()
    elif (not robot.info.gyroscope.is_upside_down) and Flipped:
       print("Flipping!")
       Flipped = False
       any_fails = True
       robot.flip_coordinates() 

    robot_vector = get_bearing_vec(robot.info.gyroscope.forward)
    
    cur_bearing = get_hillless_bearing(robot_vector)  
    
    robot_gps = robot.info.gps.position
    current_gps = np.array([robot_gps.x,robot_gps.y,robot_gps.z])
    
    now = time.time()
    current_speed = vec_dist(last_gps,current_gps)/(now-then)

    if current_speed < 1.5 and (outside_dir_margin(cur_bearing,target_bearing,2*math.pi/3) or (outside_dir_margin(cur_bearing,target_bearing,math.pi/3) and Pirouetting)):
       pirouette(cur_bearing,target_bearing)
       Pirouetting = True  # To this back to True and fix this spin up!
       might_print(99,"Cur_B = "+str(cur_bearing)+"  Tar_B =" + str(target_bearing)+ "   Mar = " + str(2*margin)+" Speed= ",current_speed)
       
    else:
       Pirouetting = False
       keep_speed(current_speed,cur_bearing,target_bearing,margin)
       steer_control(current_speed,cur_bearing,target_bearing,margin)

    then = now
    last_gps = current_gps

# ---------------------------------------
def Reverse_away_if_necessary():

    print("REVERSING!")

    enemy = robot.info.radar.it_ping

    IT_bearing = get_bearing(get_bearing_vec(enemy))

    en_dist = get_dist_to_foe(enemy)

    behind = behind_me(IT_bearing)

    if not behind and en_dist < 5:
       steer(.5)  # ideally you'd be smarter about which way to turn here
       set_torque(-max_torque*1.1)
       time.sleep(1)
    

#-----------------------------------------
def which_opponent_closest(enemies_arr):

    return 0


#-----------------------------------------
def total_lidar_space(lidar,start_row,past_last_row,start_col,past_last_col):

    temp = np.array(lidar[start_row:past_last_row,start_col:past_last_col])

    return np.sum(temp)
    
#-----------------------------------------
def smooth_test_lidar_space(lidar,start_row,past_last_row,start_col,past_last_col):

    temp = np.array(lidar[start_row:past_last_row,start_col:past_last_col])
    sum = total_lidar_space(lidar,start_row,past_last_row,start_col,past_last_col)
    temp_std = np.std(temp)
    
    if temp_std == 0:
       temp_std = sum/500
       
    return sum/temp_std
    
#-----------------------------------------
def bias_off_roll_and_clear_space(goal_dir):

    roll_adj = (-1 * roll())/15
    
    lidar = np.array(get_front_high_lidar())

    left_total_space = total_lidar_space(lidar,3,7,0,6)
    right_total_space = total_lidar_space(lidar,3,7,6,12)
    net_space = (right_total_space - left_total_space) / (right_total_space + left_total_space)

    left_amount_smooth = smooth_test_lidar_space(lidar,3,7,0,6)
    right_amount_smooth = smooth_test_lidar_space(lidar,3,7,6,12)
    might_print(2,"LAS = " + str(left_amount_smooth) + " and RAS = ",right_amount_smooth)
    net_smooth = (right_amount_smooth - left_amount_smooth) / (right_amount_smooth + left_amount_smooth)

    lidar_bias = (net_space + net_smooth)/25
    might_print(2,"Lidar bias on direction = " + str(lidar_bias) + " and adj for roll is ",roll_adj)

    adj = rad_safe_add(goal_dir, lidar_bias + roll_adj)

    return adj

#-----------------------------------------
def  Play_Tag():

  global goal_direction
  
  LAST_IT = robot.info.isIt

  while True:

    IT = robot.info.isIt

    if IT:
       index = which_opponent_closest(robot.info.radar.pings)
       goal_direction = charge_enemy(robot.info.radar.pings[index])  # if there is more than one you should go after the closest!
    else:
       goal_direction = flee_enemy(robot.info.radar.it_ping)

    hold_bearing_and_speed(goal_direction, .2)

    if not IT and LAST_IT:
       Reverse_away_if_necessary()

    LAST_IT = IT
        
    time.sleep(0.0011)


# ---------------------------------------
def Fight():

  global goal_direction
  
  while True:

    if len(robot.info.radar.pings) > 0:
       index = which_opponent_closest(robot.info.radar.pings)
       goal_direction = charge_enemy(robot.info.radar.pings[index])  # if there is more than one you should go after the closest!

    goal_direction = bias_off_roll_and_clear_space(goal_direction)
        
    hold_bearing_and_speed(goal_direction, .2)
        
    time.sleep(0.0011)


# ---------------------------------------
def its_been_x_since_a_save(delay):

    global savetime

    current = int(round(time.time()*1000))

    return (current - savetime) > 1000*delay  # i.e. its been more than 10 seconds

# ---------------------------------------
def savemaps():

    global elevation_high_map
    global avg_speed_map
    global num_fails_map
    global num_visits_map
    
    np.savetxt("elevation_high_map.txt",elevation_high_map)
    np.savetxt("avg_speed_map.txt",avg_speed_map)
    np.savetxt("num_fails_map.txt",num_fails_map)
    np.savetxt("num_visits_map.txt",num_visits_map)


# ---------------------------------------
def update_map():

    global elevation_high_map
    global avg_speed_map
    global num_fails_map
    global num_visits_map
    global last_speed
    global any_fails

    x = round((robot.info.gps.position.x)/100) + half_map_width
    y = round((robot.info.gps.position.y)/100) + half_map_height

    cur_el = robot.info.gps.position.z
    cur_speed = last_speed
    
    num_visits = num_visits_map[x][y]
    
    old_avg_speed = avg_speed_map[x][y]
    new_avg_speed = (old_avg_speed*num_visits + cur_speed) / (num_visits+1)
    avg_speed_map[x][y] = new_avg_speed
    
    if cur_el > elevation_high_map[x][y]:
       elevation_high_map[x][y] = cur_el

    if any_fails:
       num_fails_map[x][y] += 1
       any_fails = False

    num_visits_map[x][y] += 1
    

# ---------------------------------------   
def Explore():

  global savetime
  global goal_direction

  while True:

    goal_direction = bias_off_roll_and_clear_space(goal_direction)
        
    hold_bearing_and_speed(goal_direction, .2)

    #update_map()
    
    """if (last_speed < 2) and (its_been_x_since_a_save(20)) or its_been_x_since_a_save(90):
        savemaps()
        savetime = int(round(time.time()*1000))
    else:"""
    time.sleep(0.0011)
    
# ----------- MAIN LOOP! ----------------

#setup_maps(start_maps_over)

if robot.info.gamemode == "Freeplay":
   Fight()

elif robot.info.gamemode == "Singleplayer":
   Explore()
   
else:   # "Classic Tag" 
   Play_Tag()

     
robot.disconnect()


