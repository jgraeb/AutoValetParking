import numpy as np
import sys
sys.path.append('..')
current_time = 0

pedestrians_to_keep= set()
cars_to_show = []
pedestrians_to_show = []
###
average_arrival_rate = 0.1 # per second
beta = 1/average_arrival_rate
average_park_time = 50 # seconds
MAX_BUFFER_SIZE = np.inf
MAX_NO_PARKING_SPOTS = 2
OPEN_TIME = 500 # not yet working
TARGET_SPEED = 10/3.6 # 10 km/h
meters = 90
SCALE_FACTOR_SIM = 3428/meters # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = meters/300 # scale from pixels to meters in grid planner
# Dropoff position
START_X = 140*SCALE_FACTOR_PLAN # m
START_Y = 55*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
DROP_OFF = (START_X,START_Y,0)
# Pickup Position
PICK_UP = (200,50,0,0) # in pixels
# Pedestrian Pathway Dropoff
start_walk_lane = (1600,509)
end_walk_lane = (3412,509)
# Pedestrian lower crosswalk
start_walk_lane_2 = (3428,2088)
end_walk_lane_2 = (458,2088)
