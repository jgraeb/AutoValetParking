#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Aug 2019

@author: Jiaqi Yan (jiaqi@caltech.edu)

"""

import imageio
import random
import os
import numpy as np
from numpy import cos, sin, arctan2, pi
from PIL import Image
import scipy.integrate as integrate
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append('..')
from prepare.queue import Queue

medic = dir_path + '/imglib/pedestrians/medic' + '.png'
all_pedestrian_types = {'1','2','3','4','5','6'}

# pedestrian class
class Pedestrian:
    def __init__(self,
                 init_state = [2908,665,-pi/2,0], # (x, y, theta, gait)
                 number_of_gaits = 6,
                 gait_length = 4,
                 gait_progress = 0,
                 film_dim = (1, 6),
                 prim_queue = None, # primitive queue
                 pedestrian_type = '3',
                 ):
        # init_state: initial state by default 
        #self.vee_max = 50
        self.alive_time = 0
        self.is_dead = False
        self.state = np.array(init_state, dtype="float")
        self.number_of_gaits = film_dim[0] * film_dim[1]
        self.gait_length = gait_length
        self.gait_progress = gait_progress
        self.film_dim = film_dim
        self.pedestrian_type = random.choice(['1','2','3','4','5','6'])
        if prim_queue == None:
            self.prim_queue = Queue()
        else:
            prim_queue = prim_queue
        self.fig = dir_path + '/imglib/pedestrians/walking' + pedestrian_type + '.png'

    def next(self, inputs, dt):
        """
        The pedestrian advances forward
        """
        if self.is_dead:
            if self.fig != medic:
                self.fig = medic
        else:
            dee_theta, vee = inputs
            self.state[2] += dee_theta # update heading of pedestrian
            self.state[0] += vee * cos(self.state[2]) * dt # update x coordinate of pedestrian
            self.state[1] += vee * sin(self.state[2]) * dt # update y coordinate of pedestrian
            distance_travelled = vee * dt # compute distance travelled during dt
            gait_change = (self.gait_progress + distance_travelled / self.gait_length) // 1 # compute number of gait change
            self.gait_progress = (self.gait_progress + distance_travelled / self.gait_length) % 1
            self.state[3] = int((self.state[3] + gait_change) % self.number_of_gaits)
            self.alive_time += dt

    def extract_primitive(self):
       #TODO: rewrite the comment below
       """
       This function updates the primitive queue and picks the next primitive to be applied. When there is no more primitive in the queue, it will
       return False

       """
       while self.prim_queue.len() > 0:
           if self.prim_queue.top()[1] < 1: # if the top primitive hasn't been exhausted
               prim_data, prim_progress = self.prim_queue.top() # extract it
               return prim_data, prim_progress
           else:
               self.prim_queue.pop() # pop it
       return False

    def prim_next(self, dt):
        if self.extract_primitive() == False: # if there is no primitive to use
            self.next((0, 0), dt)
        else:
            prim_data, prim_progress = self.extract_primitive() # extract primitive data and primitive progress from prim
            start, finish, vee = prim_data # extract data from primitive
            total_distance,_ = self.get_walking_displacement(start, finish)
            if prim_progress == 0: # ensure that starting position is correct at start of primitive
                self.state[0] = start[0]
                self.state[1] = start[1]
            if start == finish: #waiting mode
                remaining_distance = 0
                self.state[3] = 0 # reset gait
                if self.prim_queue.len() > 1: # if current not at last primitive
                    last_prim_data, last_prim_progress = self.prim_queue.bottom() # extract last primitive
                    _, last_finish, vee = last_prim_data
                    _,heading = self.get_walking_displacement(self.state, last_finish)
                    if self.state[2] != heading:
                        self.state[2] = heading
            else: # if in walking mode
                remaining_distance,heading = self.get_walking_displacement(self.state,finish)
                if self.state[2] != heading:
                    self.state[2] = heading
            if vee * dt > remaining_distance and remaining_distance != 0:
                self.next((0, remaining_distance/dt), dt)
            else:
                self.next((0, vee), dt)
            if total_distance != 0:
                prim_progress += dt / (total_distance / vee)
            self.prim_queue.replace_top((prim_data, prim_progress)) # update primitive queue
            
    def get_walking_displacement(self, start, finish):
        dx = finish[0] - start[0]
        dy = finish[1] - start[1]
        distance = np.linalg.norm(np.array([dx, dy]))
        heading = arctan2(dy,dx)
        return distance, heading

    