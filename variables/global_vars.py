# Global Variables for Automated Valet Parking

import numpy as np
import sys
sys.path.append('..')
current_time = 0

#Gazebo/ros parameters
catkin_ws_folder = "/home/tom/SURF/catkin_ws_surf"
nbr_of_robots = 2
nbr_of_pedestrians = 2
nbr_of_obstacles = 2
GROUND_PLANE_LENGTH = 90/14 # [m]
GROUND_PLANE_PIXELS_HRES = 2866
GROUND_PLANE_PIXELS_LRES = 250
GROUND_PLANE_MPP_LRES = GROUND_PLANE_LENGTH/GROUND_PLANE_PIXELS_LRES


###
# Car parking parameters
average_arrival_rate = 0.1 # per second
beta = 1/average_arrival_rate
average_park_time = 300 # seconds

# Communication channel parameters
MAX_BUFFER_SIZE = np.inf # infinite buffer size

# Parking lot parameters
MAX_NO_PARKING_SPOTS = 25 # currently globals_vars.parking_data.parking_spots used instead
OPEN_TIME = 500 # garage closes after that time, not yet used
TARGET_SPEED = 10/3.6 # Max. speed in the lot - 10 km/h
TOW_TIME = 100 # cars are towed after that time
DELAY_THRESH = 0 # after delay exceeds this time, unparking areas are reserved

# Scaling the topology
meters = 90 #x-direction
SCALE_FACTOR_SIM = 3428/meters # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = meters/300 # scale from pixels to meters in grid planner


# Parking lot locations of interest
# Dropoff position
START_X = 140*SCALE_FACTOR_PLAN # m
START_Y = 55*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
DROP_OFF = (START_X,START_Y,START_YAW,0) # (x,y,yaw,vel)
DROP_OFF_PIX = (140,55,0,0)

START_X_GAZEBO = 100*GROUND_PLANE_MPP_LRES # m
START_Y_GAZEBO = 55*GROUND_PLANE_MPP_LRES # m
START_YAW_GAZEBO = 0 # rad
DROP_OFF_GAZEBO = (START_X,START_Y,START_YAW,0) # (x,y,yaw,vel)
DROP_OFF_PIX_GAZEBO = (100,55,0,0)

# Pickup Position
PICK_UP = (260,60,0,0) # in pixels
PICK_UP_GAZEBO = (196,60,0,0) # in pixels

# Pedestrian Pathway Dropoff
start_walk_lane = (1600,509)
end_walk_lane = (3412,509)
start_walk_lane_gazebo = (1100,509)
end_walk_lane_gazebo = (2640,509)

# Pedestrian lower crosswalk
start_walk_lane_2 = (458,2090)
end_walk_lane_2 = (3428,2090)
start_walk_lane_2_gazebo = (630,2090)
end_walk_lane_2_gazebo = (2866,2090)

