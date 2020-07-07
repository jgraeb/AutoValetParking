# Automated Valet Parking - Static obstacle for testing
# Josefine Graebener
# California Institute of Technology
# July, 2020

from prepare.boxcomponent import BoxComponent
import trio
from variables.global_vars import *
import numpy as np


class Obstacles(BoxComponent):
    def __init__(self,
                 loc = [110,80,10], # (x, y, radius) in gridpoints
                 ):
        # init_state: initial state by default 
        self.name = 'Obstacles'
        self.obs = dict()
        self.num_obs = len(self.obs) # number of obstacles currently in the lot
        self.max_serial_number = 0
        #self.location = np.array(loc, dtype="float")

    def create_obstacle_map(self):
        self.obs = {1: (170, 100, 0, 3), 
        2: (180, 100, 0, 3), 
        3: (190, 100, 0, 3)}
        self.num_obs = len(self.obs)
        self.max_serial_number = self.max_serial_number + self.num_obs
        print('Obstacle Map created')
        return self.obs

    def add_obstacle(self,obs):
        num = self.max_serial_number
        self.obs.update({num : obs})
        print('Obstacle added')

    def rmv_obstacle(self,obsnum):
        self.obs.pop(obsnum)
        print('Obstacle removed')

    def get_updated_obstacles(self):
        return self.obs

