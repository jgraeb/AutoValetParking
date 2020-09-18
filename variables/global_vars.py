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
GROUND_PLANE_PPM_HRES = GROUND_PLANE_PIXELS_HRES/GROUND_PLANE_LENGTH
back_to_sim_front_wheel_length = 0.22

turtlebotCtrl = False          
velCtrl = False               
accSteerCtrl = True        
accRotVelCtrl = False       

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]           
MAX_DSTEER = np.deg2rad(40.0)  # maximum steering speed [rad/s]            
MAX_SPEED = 0.7  # maximum speed [m/s]                                    
MIN_SPEED = -0.5  # minimum speed [m/s]                                     
MAX_ACCEL = 0.1  # maximum accel [m/ss]                                    


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
TARGET_SPEED = 0.25 # Max. speed in the lot - 10 km/h, for gazebo: 0.14m/S
TOW_TIME = 100 # cars are towed after that time
DELAY_THRESH = 0 # after delay exceeds this time, unparking areas are reserved

# Scaling the topology
meters = 90 #x-direction
SCALE_FACTOR_SIM = GROUND_PLANE_PPM_HRES # prev 3428/meters # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = GROUND_PLANE_MPP_LRES # prev meters/300 # scale from pixels to meters in grid planner


# Parking lot locations of interest
# Dropoff position
START_X = 100*SCALE_FACTOR_PLAN # m
START_Y = 60*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
DROP_OFF = (START_X,START_Y,START_YAW,0) # (x,y,yaw,vel)
DROP_OFF_PIX = (100,55,0,0)


# Pickup Position
PICK_UP = (196,60,0,0) # in pixels

# Pedestrian Pathway Dropoff
start_walk_lane = (1100,509)
end_walk_lane = (2640,509)

# Pedestrian lower crosswalk
start_walk_lane_2 = (630,2090)
end_walk_lane_2 = (2866,2090)