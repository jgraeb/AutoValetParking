# Global Variables for Automated Valet Parking

import numpy as np
import sys
sys.path.append('..')
current_time = 0

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
meters = 90
SCALE_FACTOR_SIM = 3428/meters # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = meters/300 # scale from pixels to meters in grid planner

# Parking lot locations of interest
# Dropoff position
START_X = 140*SCALE_FACTOR_PLAN # m
START_Y = 55*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
DROP_OFF = (START_X,START_Y,START_YAW,0) # (x,y,yaw,vel)
# Pickup Position
PICK_UP = (260,60,0,0) # in pixels
# Pedestrian Pathway Dropoff
start_walk_lane = (1600,509)
end_walk_lane = (3412,509)
# Pedestrian lower crosswalk
start_walk_lane_2 = (458,2090)
end_walk_lane_2 = (3428,2090)
