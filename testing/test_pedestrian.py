# Automated Valet Parking - Environment Pedestrian Component
# Josefine Graebener, Jiaqi Yan, Tung Phan
# California Institute of Technology
# July, 2020

from prepare.boxcomponent import BoxComponent
import trio
from variables.global_vars import start_walk_lane, end_walk_lane
from prepare.queue import Queue
import random
import numpy as np
from ipdb import set_trace as st

(2700,2090)

class TestPed(BoxComponent):
    def __init__(self,
                 init_state = [2700,2090,np.pi/2,0], # (x, y, theta, gait)
                 number_of_gaits = 6,
                 gait_length = 4,
                 gait_progress = 0,
                 film_dim = (1, 6),
                 prim_queue = None, # primitive queue
                 pedestrian_type = '3',
                 ):
        # init_state: initial state by default
        self.name = 'Pedestrian {}'.format(id(self))
        self.state = np.array(init_state, dtype="float")
        self.alive_time = 0
        self.is_dead = False
        self.number_of_gaits = film_dim[0] * film_dim[1]
        self.gait_length = gait_length
        self.gait_progress = gait_progress
        self.film_dim = film_dim
        self.pedestrian_type = random.choice(['1','2','3','4','5','6'])
        if prim_queue == None:
            self.prim_queue = Queue()
        else:
            prim_queue = prim_queue
        self.fig =  '../imglib/pedestrians/walking' + pedestrian_type + '.png'
        self.medic = '../imglib/pedestrians/medic' + '.png'
        self.all_pedestrian_types = {'1','2','3','4','5','6'}
        self.dt = 0.1

    async def next(self, inputs, dt):
        """
        The pedestrian advances forward
        """
        #await trio.sleep(0)
        if self.is_dead:
            if self.fig != self.medic:
                self.fig = self.medic
        else:
            dee_theta, vee = inputs
            self.state[2] += dee_theta # update heading of pedestrian
            self.state[0] += vee * np.cos(self.state[2]) * self.dt # update x coordinate of pedestrian
            self.state[1] += vee * np.sin(self.state[2]) * self.dt # update y coordinate of pedestrian
            distance_travelled = vee * self.dt # compute distance travelled during dt
            gait_change = (self.gait_progress + distance_travelled / self.gait_length) // 1 # compute number of gait change
            self.gait_progress = (self.gait_progress + distance_travelled / self.gait_length) % 1
            self.state[3] = int((self.state[3] + gait_change) % self.number_of_gaits)
            self.alive_time += self.dt

    async def extract_primitive(self):
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

    async def prim_next(self):
        #await trio.sleep(0)
        if await self.extract_primitive() == False: # if there is no primitive to use
            await self.next((0, 0), self.dt)
        else:
            prim_data, prim_progress = await self.extract_primitive() # extract primitive data and primitive progress from prim
            start, finish, vee = prim_data # extract data from primitive
            total_distance,_ = await self.get_walking_displacement(start, finish)
            if prim_progress == 0: # ensure that starting position is correct at start of primitive
                self.state[0] = start[0]
                self.state[1] = start[1]
            if start == finish: #waiting mode
                remaining_distance = 0
                self.state[3] = 0 # reset gait
                if self.prim_queue.len() > 1: # if current not at last primitive
                    last_prim_data, last_prim_progress = self.prim_queue.bottom() # extract last primitive
                    _, last_finish, vee = last_prim_data
                    _,heading = await self.get_walking_displacement(self.state, last_finish)
                    if self.state[2] != heading:
                        self.state[2] = heading
            else: # if in walking mode
                remaining_distance,heading = await self.get_walking_displacement(self.state,finish)
                if self.state[2] != heading:
                    self.state[2] = heading
            if vee * self.dt > remaining_distance and remaining_distance != 0:
                await self.next((0, remaining_distance/self.dt), self.dt)
            else:
                await self.next((0, vee), self.dt)
            if total_distance != 0:
                prim_progress += self.dt / (total_distance / vee)
            self.prim_queue.replace_top((prim_data, prim_progress)) # update primitive queue

    async def get_walking_displacement(self, start, finish):
        dx = finish[0] - start[0]
        dy = finish[1] - start[1]
        distance = np.linalg.norm(np.array([dx, dy]))
        heading = np.arctan2(dy,dx)
        return distance, heading

    async def start_ped(self,start_walk, end_walk):
        print('Start Pedestrian')
        self.start_walk_lane = start_walk
        self.end_walk_lane = end_walk
        self.prim_queue.enqueue(((self.start_walk_lane, self.end_walk_lane, 60), 0))
        print('Pedestrian is walking from {0} to {1}'.format(self.start_walk_lane,self.end_walk_lane))

    async def ped_walk(self):
        if self.status == 'WalkE':
            while (self.state[0] < self.end_walk_lane[0]): # if not at the destination
                # print(self.state)
                if self.status == 'Stop':
                    while self.status == 'Stop':
                        await trio.sleep(3)
                elif self.status == 'Delete':
                    break
                await self.prim_next()
                await trio.sleep(0)
        elif self.status =='WalkW':
            while (self.state[0] > self.end_walk_lane[0]): # if not at the destination
                # print(self.state)
                if self.status == 'Stop':
                    while self.status == 'Stop':
                        await trio.sleep(3)
                elif self.status == 'Delete':
                    break
                await self.prim_next()
                await trio.sleep(0)
            await trio.sleep(0)

    async def listen_for_commands(self):
        async with self.in_channels['TestSuite']:
            async for response in self.in_channels['TestSuite']:
                start_pos = response[0]
                end_pos = response[1]
                self.prim_queue.delete()
                self.start_ped(start_pos,end_pos)
                self.ped_walk()


    async def run(self, start_walk, end_walk):
        await self.start_ped(start_walk,end_walk)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.ped_walk)
            await trio.sleep(0)
