import curses
from ipdb import set_trace as st
import numpy as np
import trio
import random
import math
import pdb
# Animation
import sys
sys.path.append('..') # enable importing modules from an upper directory:
sys.path.append('../demo') # enable importing modules from demo
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from prepare.helper import draw_car, draw_pedestrian
from prepare.queue import Queue
from component import parking_lot
#import component.pedestrian as Pedestrian
from variables.data import parking_spots, pathspot0
# planning
import _pickle as pickle
#from motionplanning.tools import astar_trajectory
import motionplanning.end_planner as path_planner
# tracking
import motiontracking.mpc_tracking as tracking

average_arrival_rate = 0.5 # per second
beta = 1/average_arrival_rate
average_park_time = 200 # seconds
MAX_BUFFER_SIZE = np.inf
MAX_NO_PARKING_SPOTS = 1
OPEN_TIME = 500 # not yet working
ped_startpos = (2908,665)
ped_endpos = (3160,665)
TARGET_SPEED = 10/3.6 # 10 km/h
SCALE_FACTOR_SIM = 3173/90 # scale from meters to pixels in simulation
SCALE_FACTOR_PLAN = 90/300 # scale from pixels to meters in grid planner
START_X = 120*SCALE_FACTOR_PLAN # m
START_Y = 60*SCALE_FACTOR_PLAN # m
START_YAW = 0 # rad
start_walk_lane = (1287,665)
end_walk_lane = (3160,665)

def get_current_time():
    return trio.current_time() - start_time

def create_unidirectional_channel(sender, receiver, max_buffer_size, name = False):
    out_channel, in_channel = trio.open_memory_channel(max_buffer_size)
    if name:
        sender.out_channels[name] = out_channel
        receiver.in_channels[name] = in_channel
    else:
        sender.out_channels[receiver.name] = out_channel
        receiver.in_channels[sender.name] = in_channel

def create_bidirectional_channel(compA, compB, max_buffer_size):
    create_unidirectional_channel(sender=compA, receiver=compB, max_buffer_size=max_buffer_size)
    create_unidirectional_channel(sender=compB, receiver=compA, max_buffer_size=max_buffer_size)

class CarInfo:
    def __init__(self, arrive_time, depart_time):
        self.arrive_time = arrive_time
        self.depart_time = depart_time

class BoxComponent:
    def __init__(self):
        self.in_channels = dict()
        self.out_channels = dict()
        

class Simulation(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.ax = []
        self.fig = []
        self.background = []
        self.start_walk_lane = (2908,665)
        self.end_walk_lane = (3160,665)
    
    async def add_car_to_sim(self):
        while True:
            async with self.in_channels['Game']:
                async for car in self.in_channels['Game']:
                    print('Simulation System - Adding new car to Map')
                    self.cars.append(car)

    async def add_ped_to_sim(self):
        while True:
            async with self.in_channels['PedSimulation']:
                async for ped in self.in_channels['PedSimulation']:
                    print('Simulation System - Adding new pedestrian to Map')
                    self.peds.append(ped)
                    #print(self.peds)

    def animate(self, frame_idx): # update animation by dt
        self.ax.clear()
        # scale to the large topo
        xoffset = 0
        yoffset = 200
        for pedestrian in self.peds:
            draw_pedestrian(pedestrian,self.background)
        for car in self.cars:
            draw_car(self.background, car.x*SCALE_FACTOR_SIM+xoffset,car.y*SCALE_FACTOR_SIM+yoffset,car.yaw)
        # to check parking spot locations
        #for key,value in parking_spots.items():
        #    draw_car(self.background,  value[0]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+xoffset, value[1]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+yoffset,np.deg2rad(value[2]))
        the_parking_lot = [self.ax.imshow(self.background)] # update the stage
        self.background.close()
        self.background = parking_lot.get_background()
        all_artists = the_parking_lot
        return all_artists

    async def update_simulation(self):
        self.fig = plt.figure()
        while True:
            self.ax = self.fig.add_axes([0,0,1,1]) # get rid of white border
            plt.axis('off')
            self.background = parking_lot.get_background()
            ani = animation.FuncAnimation(self.fig, self.animate, frames=1, interval=1**3, blit=True, repeat=False)
            plt.pause(0.001)
            plt.draw()
            await trio.sleep(0)
            print('------------Figure updating-------------')

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.add_car_to_sim)
            await trio.sleep(0)
            nursery.start_soon(self.add_ped_to_sim)
            await trio.sleep(0)
            nursery.start_soon(self.update_simulation)
            await trio.sleep(0)

class Game(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.lot_status = []

    async def keep_track_influx(self):
        async with self.in_channels['GameEnter']:
            async for car in self.in_channels['GameEnter']:
                print('Game System - Adding new car to Game')
                await self.out_channels['Enter'].send(car)
                await self.out_channels['Simulation'].send(car)
                self.cars.append(car)

    async def keep_track_outflux(self):
        async with self.in_channels['GameExit']:
            async for car in self.in_channels['GameExit']:
                print('Game System - Removing car to Game')
                self.cars.remove(car)
                await self.out_channels['Exit'].send(car)

    async def keep_track_influx_peds(self):
        async with self.in_channels['GameEnterPeds']:
            async for pedestrian in self.in_channels['GameEnterPeds']:
                print('Game System - Adding new pedestrian to Game')
                #await self.out_channels['PedEnter'].send(pedestrian) # for Map
                await self.out_channels['PedSimulation'].send(pedestrian)
                self.peds.append(pedestrian)

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)


class Map(BoxComponent): 
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.car_positions = dict()

    async def send_position(self,car):
        print('Map System - sending car position to Planner')
        for key, value in self.car_positions.items():
            if (key==car.name):
                pos = (value.x , value.y, value.yaw)
        await self.out_channels['Planner'].send((car,pos))

    async def add_car_to_map(self):
        async with self.in_channels['Enter']:
            async for car in self.in_channels['Enter']:
                print('Map System - Adding new car to Map')
                self.car_positions.update({car.name: (car)})
                print(self.car_positions)

    async def send_camera_data_to_planner(self):
        async with self.in_channels['Planner']:
            async for car in self.in_channels['Planner']:
                await self.send_position(car)
                print(self.car_positions)
            
    async def rmv_car_from_map(self):
        async with self.in_channels['Exit']:
            async for car in self.in_channels['Exit']:
                print('Map System - Removing {} from Map'.format(car.name))
                self.car_positions.pop(car.name)
                print(self.car_positions)

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.add_car_to_map)
            nursery.start_soon(self.rmv_car_from_map)
            nursery.start_soon(self.send_camera_data_to_planner)

    
class Planner(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.nursery = nursery
        self.cars = []

    async def get_car_position(self,car):
        print('Planner - Sending position request to Map system')
        await self.out_channels['Map'].send(car)
        car, pos = await self.in_channels['Map'].receive()
        return pos

    async def find_spot_coordinates(self, spot):
        return parking_spots[spot]

    async def send_directive_to_car(self, car, end):
        pos = await self.get_car_position(car)
        start = np.zeros(4)
        start[0] = pos[0]/SCALE_FACTOR_PLAN
        start[1] = pos[1]/SCALE_FACTOR_PLAN
        start[2] = np.rad2deg(pos[2])
        # get trajectory from Planning Graph
        traj = await self.get_path(start,end) 
        print('Planner - sending Directive to {0}'.format(car.name))
        await self.out_channels[car.name].send(traj)
        await trio.sleep(1)

    async def get_path(self, start, end): 
        sys.path.append('../motionplanning')
        with open('planning_graph_refined.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        traj = path_planner.get_mpc_path(start,end,planning_graph)
        return traj

    async def update_car_response(self, receive_response_channel):
        async with receive_response_channel:
            async for response in receive_response_channel:
                car = response[0]
                resp = response[1]
                print('Planner - receiving "{0}" response from {1}'.format(resp,car.name))
                await trio.sleep(0)
                await self.send_response_to_supervisor(car,resp)

    async def send_response_to_supervisor(self,car,response):
        print('Planner - sending "{0} - {1}" response to Supervisor'.format(response,car.name))
        await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)
    
    async def check_supervisor(self,send_response_channel):
        async for directive in self.in_channels['Supervisor']:
            car = directive[0]
            todo = directive[1]
            if todo[0] == 'Park':
                print('Planner - receiving directive from Supervisor to park {0} in Lot {1}'.format(car.name,todo[1]))
                self.cars.append(car)
                create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                self.nursery.start_soon(car.run,send_response_channel)
                end = await self.find_spot_coordinates(todo[1])
                await self.send_directive_to_car(car, end)
            elif todo == 'Pickup':
                print('Planner -  receiving directive from Supervisor to retrieve {0}'.format(car.name))
                end = [2640,774,0]
                await self.send_directive_to_car(car, end)

    async def run(self):
        send_response_channel, receive_response_channel = trio.open_memory_channel(25)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.check_supervisor,send_response_channel)
            await trio.sleep(1)
            nursery.start_soon(self.update_car_response,receive_response_channel)

class Pedestrian(BoxComponent):
    def __init__(self,
                 init_state = [start_walk_lane[0],start_walk_lane[1],-np.pi/2,0], # (x, y, theta, gait)
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
        if await self.extract_primitive() == False: # if there is no primitive to use
            self.next((0, 0), self.dt)
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

    async def ped_walk(self):
        while (self.state[0] < self.end_walk_lane[0]): # if not at the destination
            print('Pedestrian is walking')
            print(self.state)
            await self.prim_next()
            await trio.sleep(0)

    async def run(self, start_walk, end_walk):
        await self.start_ped(start_walk, end_walk)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.ped_walk)
            await trio.sleep(0)

class Car(BoxComponent):
    def __init__(self, arrive_time, depart_time):
        super().__init__()
        self.name = 'Car {}'.format(id(self))
        self.arrive_time = arrive_time
        self.depart_time = depart_time
        self.ref = None
        self.x = START_X
        self.y = START_Y
        self.yaw = START_YAW
        self.v = 0.0

    async def update_planner_command(self,send_response_channel):
        async for directive in self.in_channels['Planner']:
            self.ref = directive
            print('{0} - Receiving Directive from Planner'.format(self.name))
            await self.track_reference(self.ref)
            await trio.sleep(0)
            await self.send_response(send_response_channel)

    async def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        if oa is None or od is None:
            oa = [0.0] * tracking.T
            od = [0.0] * tracking.T
        for i in range(tracking.MAX_ITER):
            xbar = tracking.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, _, _, _, _ = tracking.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= tracking.DU_TH:
                break
        return oa, od

    async def track_async(self, cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed): # modified from MPC
        goal = [cx[-1], cy[-1]]
        tracking.state = initial_state
        # initial yaw compensation
        if tracking.state.yaw - cyaw[0] >= math.pi:
            tracking.state.yaw -= math.pi * 2.0
        elif tracking.state.yaw - cyaw[0] <= -math.pi:
            tracking.state.yaw += math.pi * 2.0
        time = 0.0
        x = [tracking.state.x]
        y = [tracking.state.y]
        yaw = [tracking.state.yaw]
        v = [tracking.state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = tracking.calc_nearest_index(tracking.state, cx, cy, cyaw, 0)
        odelta, oa = None, None
        cyaw = tracking.smooth_yaw(cyaw)
        while tracking.MAX_TIME >= time:
            xref, target_ind, dref = tracking.calc_ref_trajectory(tracking.state, cx, cy, cyaw, ck, sp, dl, target_ind)
            x0 = [tracking.state.x, tracking.state.y, tracking.state.v, tracking.state.yaw]  # current state
            oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
            if odelta is not None:
                di, ai = odelta[0], oa[0]
            tracking.state = tracking.update_state(tracking.state, ai, di)
            time = time + tracking.DT
            self.x = tracking.state.x
            self.y = tracking.state.y
            self.yaw = tracking.state.yaw
            self.v = tracking.state.v
            await trio.sleep(0)
            if tracking.check_goal(tracking.state, goal, target_ind, len(cx),goalspeed): # modified goal speed
                break
 
    async def track_reference(self,ref):
        print('{0} - Tracking reference...'.format(self.name))
        ck = 0 
        dl = 1.0  # course tick
        for i in range(0,len(ref)-1):
            #print('Going to'+str(ref[i,:]))
            path = ref[:][i]
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            state = np.array([self.x, self.y,self.yaw])
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,TARGET_SPEED,1)
            initial_state = tracking.State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,TARGET_SPEED)
            await trio.sleep(0)
        state = np.array([self.x, self.y,self.yaw])
        path = ref[:][-1]
        cx = path[:,0]*SCALE_FACTOR_PLAN
        cy = path[:,1]*SCALE_FACTOR_PLAN
        cyaw = np.deg2rad(path[:,2])*-1
        initial_state = tracking.State(x=state[0], y=state[1], yaw=state[2], v=self.v)
        sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,0.0,1)
        await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = 'Completed'
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def run(self,send_response_channel):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command,send_response_channel)
            await trio.sleep(0)


class Supervisor(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.nursery = nursery
        self.spot_no = MAX_NO_PARKING_SPOTS
        self.parking_spots = dict([ (i, ("Vacant", "None")) for i in range(0,self.spot_no) ])

    async def send_directive_to_planner(self, car,ref):
        directive = [car,ref]
        await self.out_channels['Planner'].send(directive)

    async def update_parking_spots(self,car):
        for spot, status in self.parking_spots.items():
                if (status[1] == car.name):
                    if (status[0] == 'Assigned'):
                        self.parking_spots[spot]=(('Occupied',car.name))
                    elif (status[0] == 'Occupied'):
                        self.parking_spots[spot]=(('Vacant','None'))
                        await self.out_channels['GameExit'].send(car)
                        self.spot_no=self.spot_no+1

    async def update_planner_response(self):
        async with self.in_channels['Planner']:
            async for response in self.in_channels['Planner']:
                car = response[0]
                resp = response[1]
                print('Supervisor - receiving "{0} - {1}" Response from Planner'.format(car.name,resp))
                await self.update_parking_spots(car)
                await trio.sleep(0)
        
    def pick_spot(self,car):
        for spot, status in self.parking_spots.items():
            if (status[0]=='Vacant'):
                self.parking_spots[spot]=(('Assigned',car.name))
                return spot

    async def process_queue(self):
        async for car in self.in_channels['Customer']:
            accept_condition = False
            print(str(self.spot_no)+' parking Spots vacant')
            print(self.parking_spots)
            if self.spot_no>0:
                accept_condition = True
            if accept_condition:
                print('{} has been accepted!'.format(car.name))
                await self.out_channels['Customer'].send(True)
                self.spot_no=self.spot_no-1
                await self.out_channels['GameEnter'].send(car)
                spot = self.pick_spot(car)
                await self.send_directive_to_planner(car,('Park',spot))
                ped = Pedestrian(pedestrian_type='1')
                self.nursery.start_soon(ped.run,start_walk_lane,end_walk_lane)
                await self.out_channels['GameEnterPeds'].send(ped)

            else:
                await self.out_channels['Customer'].send(False)
                print('Garage fully occupied - A car has been rejected!')

    async def request_queue(self):
        async for car in self.in_channels['Request']:
            print('Supervisor - sending Directive to Planner to retrieve {}'.format(car.name))
            await self.send_directive_to_planner(car, 'Pickup')

    async def run(self):
        print(self.parking_spots)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.process_queue)
            nursery.start_soon(self.request_queue)
            nursery.start_soon(self.update_planner_response)


class Customer(BoxComponent):
    def __init__(self, average_arrival_rate, average_park_time):
        super().__init__()
        self.name = self.__class__.__name__
        self.average_arrival_rate = average_arrival_rate
        self.average_park_time = average_park_time
        self.cars = []

    def generate_car(self):
        arrive_time = get_current_time()
        depart_time = arrive_time + np.random.exponential(self.average_park_time)
        car = Car(arrive_time=arrive_time, depart_time=depart_time)
        return car
        
    async def run(self,end_time):
        now = trio.current_time()
        #garage_open = True
        while True:
            # spawns cars according to exponential distribution
            await trio.sleep(np.random.exponential(1/self.average_arrival_rate))
            car = self.generate_car()
            #self.cars.append(car)
            car.x = START_X
            car.y = START_Y
            car.yaw = START_YAW
            print("Car with ID {0} arrives at {1:.3f}".format(car.name, car.arrive_time))
            await self.out_channels['Supervisor'].send(car)
            accept = await self.in_channels['Supervisor'].receive() # checks if car is accepted by the garage
            if (accept == False):
                self.cars.pop()
            now = get_current_time()
            for i,cars in enumerate(self.cars): # checks for requested cars
                if cars.depart_time <= now:
                    print('{0} is requested at {1:.3f}'.format(cars.name,cars.depart_time))
                    await self.out_channels['Request'].send(cars)
                    self.cars.pop(i)
            self.cars.append(car)
            #if (now>=end_time):
            #    garage_open = False
            # Random sleeps help trigger the problem more reliably
            await trio.sleep(random.random())
        print('--------- It is {} time  --------'.format(trio.current_time()))
        print('The garage is closed now!')
            

async def main():
    global start_time
    start_time = trio.current_time()
    end_time = start_time + OPEN_TIME
    all_components = []
    print('--- Starting Parking Garage ---')
    async with trio.open_nursery() as nursery:
        map_sys = Map()
        all_components.append(map_sys)
        simulation = Simulation()
        all_components.append(simulation)
        supervisor = Supervisor(nursery = nursery)
        all_components.append(supervisor)
        game = Game()
        all_components.append(game)
        planner = Planner(nursery=nursery)
        all_components.append(planner)
        customer = Customer(average_arrival_rate = average_arrival_rate, average_park_time = average_park_time)
        # create communication channels
        create_bidirectional_channel(supervisor,planner,max_buffer_size=np.inf)
        create_bidirectional_channel(customer, supervisor, max_buffer_size=np.inf)
        create_unidirectional_channel(sender=customer, receiver=supervisor, max_buffer_size=np.inf, name='Request')
        create_unidirectional_channel(sender=game, receiver=map_sys, max_buffer_size=np.inf, name='Enter')
        create_unidirectional_channel(sender=game, receiver=map_sys, max_buffer_size=np.inf, name='Exit')
        #create_unidirectional_channel(sender=game, receiver=map_sys, max_buffer_size=np.inf, name='PedEnter')
        create_bidirectional_channel(map_sys, planner, max_buffer_size=np.inf)
        create_unidirectional_channel(sender=supervisor, receiver=game, max_buffer_size=np.inf, name='GameEnter')
        create_unidirectional_channel(sender=supervisor, receiver=game, max_buffer_size=np.inf, name='GameExit')
        create_unidirectional_channel(sender=supervisor, receiver=game, max_buffer_size=np.inf, name='GameEnterPeds')
        create_unidirectional_channel(sender=game, receiver=simulation, max_buffer_size=np.inf)
        create_unidirectional_channel(sender=game, receiver=simulation, max_buffer_size=np.inf, name='PedSimulation')
        
        for comp in all_components:
            nursery.start_soon(comp.run)
            await trio.sleep(0)
        nursery.start_soon(customer.run,end_time)

trio.run(main)
