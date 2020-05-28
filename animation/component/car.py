#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Mar 2020

@author: Jiaqi Yan

"""

import imageio
import random
import os
import numpy as np
import scipy.integrate as integrate
dir_path = os.path.dirname(os.path.realpath(__file__))
pathfile = os.path.dirname(dir_path) + '/motionplanning/nominal_trajectory.txt'

path =[]
with open(pathfile,'r') as f:
	for line in f:
		path.append(list(line.strip('\n').split(',')))


# car class
class Car:
    def __init__(self,
                 car_name,
                 init_state = [120,60,0,50], # (x, y, heading, velocity)
                 state_idx = 0
                 ):
        # init_state: initial state by default 
        #self.vee_max = 50
        self.name = car_name
        self.state = np.array(init_state, dtype="float")
        self.state_idx = state_idx
        
    def find_state(self):
        current_loc = path[self.state_idx][0].split()
        return float(current_loc[0]), float(current_loc[1]), float(current_loc[2])