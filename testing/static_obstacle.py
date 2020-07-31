# Automated Valet Parking - Static obstacles for testing
# Josefine Graebener
# California Institute of Technology
# July, 2020

from prepare.boxcomponent import BoxComponent
import trio
from variables.global_vars import *
import numpy as np
from ipdb import set_trace as st


class Obstacles(BoxComponent):
    def __init__(self,
                 loc = [110,80,10], # (x, y, radius) in gridpoints
                 ):
        super().__init__()
        # init_state: initial state by default 
        self.name = self.__class__.__name__
        self.obs = dict()
        self.num_obs = len(self.obs) # number of obstacles currently in the lot
        self.max_serial_number = 0

    def create_obstacle_map(self): # static obstacles initialized at the beginning of the simulation
        self.obs = {1: (170, 100, 0, 3), 
        2: (180, 100, 0, 3), 
        3: (190, 100, 0, 3)}
        self.num_obs = len(self.obs)
        self.max_serial_number = self.max_serial_number + self.num_obs
        print('Obstacle Map created')
        return self.obs

    def add_obstacle(self,obs): # add new obstacle
        num = self.max_serial_number
        self.obs.update({num : obs})
        self.max_serial_number = self.max_serial_number+1
        print('Obstacle added')

    def rmv_obstacle(self,obsnum): # remove an obstacle
        self.obs.pop(obsnum) 
        print('Obstacle removed')

    def get_updated_obstacles(self): # return current obstacle list
        return self.obs

    async def listen_for_commands(self,Game,Simulation,Planner):
        async with self.in_channels['TestSuite']:
            async for response in self.in_channels['TestSuite']:
                if response[0]=='Add':
                    obs = response[1]
                    self.add_obstacle(obs)
                elif response[0]=='Remove':
                    obsnum = response[1]
                    self.rmv_obstacle(obsnum)
                Planner.update_obstacle_map(Game,self,Simulation)


    async def run(self,Game,Simulation,Planner):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.listen_for_commands,Game,Simulation,Planner)
            await trio.sleep(0)

