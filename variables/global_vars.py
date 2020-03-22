import numpy as np
import sys
sys.path.append('..')
current_time = 0

#all_cars = dict()
pedestrians_to_keep= set()
cars_to_show = []
pedestrians_to_show = []

###
average_arrival_rate = 0.5 # per second
beta = 1/average_arrival_rate
average_park_time = 200 # seconds
MAX_BUFFER_SIZE = np.inf
MAX_NO_PARKING_SPOTS = 1
OPEN_TIME = 500 # not yet working
ped_startpos = (2908,665)
ped_endpos = (3160,665)
TARGET_SPEED = 10/3.6 # 10 km/h
meters = 90
SCALE_FACTOR_SIM = 3428/meters # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = meters/300 # scale from pixels to meters in grid planner
START_X = 120*SCALE_FACTOR_PLAN # m
START_Y = 60*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
start_walk_lane = (1287,665)
end_walk_lane = (3160,665)
