# Automated Valet Parking - Garage Planner Component
# Josefine Graebener
# California Institute of Technology
# March, 2020

from prepare.boxcomponent import BoxComponent
import trio
import numpy as np
import _pickle as pickle
import motionplanning.end_planner as path_planner
from prepare.communication import create_bidirectional_channel, create_unidirectional_channel, get_current_time
from variables.global_vars import SCALE_FACTOR_PLAN, PICK_UP
from variables.parking_data import parking_spots,parking_spots_original
import math
from ipdb import set_trace as st
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity

class Planner(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.nursery = nursery
        self.cars = dict()
        self.spots = dict([(i, (0)) for i in list(parking_spots.keys())])
        self.obstacles = dict()
        self.planning_graph = []
        self.original_lanes_planning_graph = []
        self.original_free_planning_graph = []
        self.planning_graph_in_use = []
        self.lanes_box = Polygon([(150,50),(150,230),(230,230),(230,50),(150,50)])
        self.reachable = np.ones((max(list(parking_spots.keys()))+1,1)) # if spots can be reached from drop_off, currently all reachable
        self.reserved_areas = dict()

    async def get_car_position(self,car):
        print('Planner - Sending position request to Map system')
        await self.out_channels['Map'].send(car)
        car, pos = await self.in_channels['Map'].receive()
        return pos

    def find_spot_coordinates(self, spot):
        return parking_spots[spot]

    async def send_directive_to_car(self, car, end, Game, original, obstacle): # directive/response system
        # if end == 'Reverse':
        #     end = (car.x/SCALE_FACTOR_PLAN-car.direction*10, car.y/SCALE_FACTOR_PLAN-car.direction*10, car.yaw, 0)
        #     print('Car moving out of the way to: '+str(end))
        pos = await self.get_car_position(car)
        start = np.zeros(4)
        start[0] = pos[0]/SCALE_FACTOR_PLAN
        start[1] = pos[1]/SCALE_FACTOR_PLAN
        start[2] = -np.rad2deg(pos[2])
        start[3] = 0
        if original:
            self.planning_graph = self.original_lanes_planning_graph
        else:
            self.get_current_planning_graph(Game)
        print('Planning - {0} driving from {1} to {2}'.format(car.name,start,end))
        traj, weight = self.get_path(start,end) 
        print('Path weight: '+str(weight))
        if traj:
            if car.replan:
                # pos = Point([(car.x/SCALE_FACTOR_PLAN,car.y/SCALE_FACTOR_PLAN)])
                # if not obstacle:
                #     dist = []
                #     for key,val in self.obstacles.items():
                #         obst = Point([(key.x/SCALE_FACTOR_PLAN,key.y/SCALE_FACTOR_PLAN)])
                #         dist.append(obst.distance(pos))
                print('Reserve Replanning Area for Car {0}!!!'.format(car.id))
                Game.reserve_replanning(car, traj, obstacle)
                #Game.check_and_reserve_other_lane(car,traj)
            if not car.replan and Game.check_and_reserve_other_lane(car,traj):
                car.hold = True
                Game.reserve_replanning(car, traj, obstacle)
            print('Planner - sending Directive to {0}'.format(car.name))
            await self.out_channels[car.name].send(traj)
        else:
            print('Planner - No Path found - sending Response to Supervisor')
            resp = ('NoPath', car)
            await self.send_response_to_supervisor(resp)
            #await self.out_channels[car.name].send('OriginalPath')
        await trio.sleep(1)

    async def add_obstacle(self, car,Game): # add failed car coordinates to obstacle list
        self.obstacles.update({car.name: (car.x,car.y,car.yaw)})
        self.update_reachability_matrix(Game)
        await self.check_obs_on_path(car, Game)

    def rmv_obstacle(self, car,Game): # remove towed car from obstacle list
        self.obstacles.pop(car.name)
        self.update_reachability_matrix(Game)

    def update_reachability_matrix(self,Game): # check which parking spots are still reachable and update self.reachable
        start = (140,55,0,0) # start position on the grid
        self.get_current_planning_graph(Game)
        for i in list(parking_spots.keys()):
            end = self.find_spot_coordinates(i)
            traj, _ = self.get_path(start,end) 
            if traj:
                self.reachable[i]=1
            else:
                self.reachable[i]=0

    async def check_obs_on_path(self,obscar, gme): # check if the failure is on the path of a car in the garage using the Game
        #if self.is_single_failure_in_acceptable_area(obscar, gme):
            obstacle = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)]).buffer(1.0) # future use car_box
            for carname in list(self.cars):
                if carname != obscar.name:
                    for car in gme.cars:
                        if carname == car.name:
                            if len(car.ref)!=0:
                                ref = car.ref
                                ref = ref[car.idx:]
                                ref = np.concatenate(ref, axis=0) 
                                ref = ref.tolist()
                                del_idx = []
                                for i in range(0,len(ref)-1): # remove duplicates
                                    if np.all(ref[i]==ref[i+1]):
                                        del_idx.append(i)
                                del_idx = np.flip(del_idx)
                                for k in del_idx:
                                    ref.pop(k)
                                line = [(entry[0],entry[1]) for entry in ref]
                                path = LineString(line)
                                if obstacle.intersects(path):
                                    obstacle = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)])
                                    if self.is_single_failure_in_acceptable_area(obscar, gme):
                                        print('FAILURE ON PATH - NEED NEW PATH CAR ID {0}'.format(car.id))
                                        car.status == 'Replan'
                                        await self.replan_path(car, gme, obstacle)
                                    else: 
                                        if not car.requested:
                                            # stop car here and replan
                                            resp = (('SpotUnreachable',obstacle), car)
                                            await self.send_response_to_supervisor(resp)
                            
    async def replan_path(self, car, Game, obstacle):
        car.status = 'Replan'
        if self.cars.get(car.name,0)=='Assigned':
            for key, val in self.spots.items(): 
                if val == car.name: 
                    spot = key
            end = self.find_spot_coordinates(spot)
            car.replan = True
            print('Car {} receives a new directive'.format(car.id))
            await self.send_directive_to_car(car, end, Game, False, obstacle)
        elif self.cars.get(car.name,0)=='Request':
            await self.send_directive_to_car(car, PICK_UP, Game, False, obstacle)

    def is_in_buffer(self,node_x,node_y,obs,): # find nodes in a buffer area around the obstacle to delete from grid
        # get car box around x,y
        buffer_back = 5
        buffer_side = 4
        buffer_front = 10
        for i in range(0,len(obs)):
            # create a buffer box around the obstacle coordinates
            x = obs[i][0]
            y = obs[i][1]
            yaw = obs[i][2]
            box = Polygon([(x-buffer_back, y+buffer_side),(x-buffer_back,y-buffer_side),(x+buffer_front,y-buffer_side),(x+buffer_front,y+buffer_side),(x-buffer_back, y+buffer_side)])
            rot_box = affinity.rotate(box, np.rad2deg(yaw), origin = (x,y))
            # add additional buffer of 1/2 gridsize to every node and check if it intersects the buffer region
            if Point(node_x,node_y).buffer(5).intersects(rot_box):
                return True
        return False

    def get_current_planning_graph(self,Game): # includes all current obstacles
        # loop through obstacles and generate new graph
        if self.obstacles:
            print('Updating Planning Graph')
            if self.is_failure_in_acceptable_area(Game):
                print('Failure is in acceptable area')
                obs = []
                obs_boxes = []
                for key,value in self.obstacles.items():
                    obs.append(value)
                    obs_boxes.append(Point(value[0]/SCALE_FACTOR_PLAN,value[1]/SCALE_FACTOR_PLAN).buffer(15.0))
                obs = [(row[0]/SCALE_FACTOR_PLAN,row[1]/SCALE_FACTOR_PLAN, row[2]) for row in obs]
                print(obs)
                # assume obs not in the acceptable area to use graph with lanes
                self.planning_graph_in_use = self.original_lanes_planning_graph
                # check if obstacle is in lanes_box (acceptable area)
                for obs_box in obs_boxes:
                    if obs_box.intersects(self.lanes_box):
                        self.planning_graph_in_use = self.original_free_planning_graph
                # find grid nodes around obstacle
                nodes = self.planning_graph_in_use['graph']._nodes
                del_nodes = [(node) for node in nodes if self.is_in_buffer(node[0],node[1],obs)]
                DEL_XY_NODES = []
                for del_node in del_nodes:
                    if (del_node[0], del_node[1]) not in DEL_XY_NODES:
                        DEL_XY_NODES.append((del_node[0], del_node[1]))
                print('del_xy_nodes:')
                print(DEL_XY_NODES)
                self.planning_graph = path_planner.update_planning_graph(self.planning_graph_in_use, DEL_XY_NODES)
                #self.planning_graph = path_planner.update_planning_graph(self.planning_graph_in_use, del_nodes)
            else:
                print('Failure blocks the narrow path')
                self.planning_graph = self.original_lanes_planning_graph
        else:
            self.planning_graph = self.original_lanes_planning_graph

    def get_path(self, start, end): # compute path using grid_planner + end_planner
        traj = None
        try:
            traj, weight = path_planner.get_mpc_path(start,end,self.planning_graph)
        except:
            st()
        if traj and not weight == np.inf:
            return traj, weight
        else: 
            traj = False
            weight = None
            return traj, weight

    def find_destination(self,car):
        if self.cars.get(car.name,0)=='Assigned':
            for key, val in self.spots.items(): 
                if val == car.name: 
                    spot = key
            end = self.find_spot_coordinates(spot)
        elif self.cars.get(car.name,0)=='Request':
            end = PICK_UP
        return end

    async def update_car_response(self, receive_response_channel,Game): # directive/response system
        async with receive_response_channel:
            async for response in receive_response_channel:
                car = response[0]
                resp = response[1]
                print('Planner - receiving "{0}" response from {1}'.format(resp,car.name))
                if response[1]=='Completed':
                    if car.reverse:
                        if self.cars.get(car.name)=='Request':
                            end = PICK_UP
                        else:
                            for key, val in self.spots.items(): 
                                if val == car.name: 
                                    spot = key
                            end = self.find_spot_coordinates(spot)
                        car.reverse = False
                        await self.send_directive_to_car(car, end,Game, False, None)
                    if self.cars.get(car.name,0)=='Assigned':
                        self.cars.update({car.name: 'Parked'})
                    elif self.cars.get(car.name,0)=='Request':
                        self.cars.pop(car.name)
                    await self.send_response_to_supervisor((resp,car))
                elif response[1]=='Failure': # car reports Failure
                    await self.add_obstacle(car,Game) # add obstacle to list
                    self.cars.update({car.name: 'Failure'})
                    await self.send_response_to_supervisor((resp,car))
                elif resp[0]=='Reverse': # make car reverse first and then compute new path
                    car.status == 'Replan'
                    obscar = resp[1]
                    await self.send_reverse(car, Game)
                    # add reversing segment
                    # print('Car {0} - Reversing first'.format(car.id))
                    # direc = [[car.x/SCALE_FACTOR_PLAN, car.y/SCALE_FACTOR_PLAN, -np.rad2deg(car.yaw)]]
                    # direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*5*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*5*np.sin(car.yaw), -np.rad2deg(car.yaw)])
                    # direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*10*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*10*np.sin(car.yaw), -np.rad2deg(car.yaw)])
                    # directive = [np.array(direc)]
                    # #st()
                    # Res_Area = Game.reserve(directive,car)
                    # self.reserved_areas.update({car.name: Res_Area})
                    # # compute path from end of reversing to destination
                    # # start = [car.x/SCALE_FACTOR_PLAN-car.direction*10*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*10*np.sin(car.yaw), car.yaw, 0]
                    # # end = self.find_destination(car)
                    # # self.get_current_planning_graph(Game)
                    # # path = self.get_path(start,end)
                    # # directive.append(path)
                    # # send directive to car and update reserved area
                    # print('Planner - sending Directive to {0}'.format(car.name))
                    # car.reverse = True
                    # await self.out_channels[car.name].send(directive)
                    # print('Reserve Replanning Area for Car {0}!!!'.format(car.id))
                    # obs = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)])
                    # try:
                    #     Game.reserve_replanning(car, directive, obs)
                    # except:
                    #     st()
                    # Game.check_and_reserve_other_lane(car,directive) 
                elif resp[0]=='Blocked': # compute a path around the blockage if possiblw, if not possible, wait
                    if self.is_failure_in_acceptable_area(Game):
                        print('Obstacles are in acceptable area.')
                        if self.cars.get(car.name,0)=='Assigned':
                            for key, val in self.spots.items(): 
                                if val == car.name: 
                                    spot = key
                            end = self.find_spot_coordinates(spot)
                            #car.replan = True
                            car.status = 'Replan'
                            car.replan = True
                            blocked_car = resp[1]
                            print('Blocked Car {0} receives a new directive'.format(car.id))
                            obs = Point([(blocked_car.x/SCALE_FACTOR_PLAN, blocked_car.y/SCALE_FACTOR_PLAN)])
                            # try to get path
                            pos = await self.get_car_position(car) # in meters
                            # convert position to grid pixels and flip angle
                            start = np.zeros(4)
                            start[0] = pos[0]/SCALE_FACTOR_PLAN
                            start[1] = pos[1]/SCALE_FACTOR_PLAN
                            start[2] = -np.rad2deg(pos[2])
                            start[3] = 0
                            self.get_current_planning_graph(Game)
                            traj, weight = self.get_path(start,end) 
                            # if no path, reverse first
                            if traj:
                                await self.send_directive_to_car(car, end, Game, False, obs)
                            else:
                                await self.send_reverse(car,Game)
                        elif self.cars.get(car.name,0)=='Request':
                            car.status = 'Replan'
                            await self.send_directive_to_car(car, PICK_UP, Game, False, None)
                    else:
                        print('No way to plan around the failure - {0} must wait'.format(car.name))
                elif resp=='RequestArea':
                    Game.reserve(None,car)
                    #await self.send_response_to_supervisor((resp,car))
                elif resp=='RequestReservedArea':
                    await self.send_response_to_supervisor((resp,car))
                elif resp[0]=='Conflict':
                    await self.send_response_to_supervisor((resp,car))
                await trio.sleep(0)
                print(self.cars)
                # await self.send_response_to_supervisor((resp,car))

    async def send_response_to_supervisor(self,resp): # directive/response system
        response = resp[0]
        car = resp[1]
        #print(resp)
        print('Planner - sending "{0} - {1}" response to Supervisor'.format(response,car.name))
        await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)
    
    async def check_supervisor(self,send_response_channel,Game, Time): # directive/response system (receiving directives from supervisor)
        async for directive in self.in_channels['Supervisor']:
            car = directive[0]
            todo = directive[1]
            print(self.spots)
            if todo[0] == 'Park':
                print('Planner - receiving directive from Supervisor to park {0} in Spot {1}'.format(car.name,todo[1]))
                self.cars.update({car.name: 'Assigned'})
                create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                self.nursery.start_soon(car.run,send_response_channel,Game, Time)
                self.spots.update({todo[1]: car.name})
                end = self.find_spot_coordinates(todo[1])
                await self.send_directive_to_car(car, end, Game, False, None)
            elif todo == 'Pickup':
                print('Planner -  receiving directive from Supervisor to retrieve {0}'.format(car.name))
                car.replan = False
                await self.send_directive_to_car(car, PICK_UP,Game, False, None)
                self.cars.update({car.name : 'Request'})
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Towed': # remove the car
                print('Planner - {0} is being towed'.format(car.name))
                self.cars.pop(car.name)
                self.rmv_obstacle(car,Game)
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Wait':
                print('Planner - {0} has to wait until path clears'.format(car.name))
                await self.send_directive_to_car(car, todo,Game, False, None)
            elif todo == 'Reverse':
                print('Planner - {0} has to drive out of the way'.format(car.name))
                st()
                await self.out_channels[car.name].send('Reverse')
            elif todo == 'OriginalPath':
                if len(car.ref) != 0: # fix here
                    await self.out_channels[car.name].send('OriginalPath')
                else:
                    if self.cars.get(car.name)=='Request':
                        end = PICK_UP
                    else:
                        try:
                            spot = self.spots.get(car.name)
                        except:
                            st()
                            for key, val in self.spots.items(): 
                                if val == car.name: 
                                    spot = key
                        end = self.find_spot_coordinates(spot)
                    await self.send_directive_to_car(car, end,Game, True, None) # original graph
            elif todo == 'Back2spot':
                print('Planner - {0} has to drive back into the spot to make space'.format(car.name))
                car.status = 'Replan'
                car.replan = True
                await self.out_channels[car.name].send('Back2spot')
            elif todo == 'ReserveReverse':
                Res_Area = Game.reserve_reverse(car)
                self.reserved_areas.update({car.name: Res_Area})

    async def send_reverse(self,car, Game):
        print('Car {0} - Reversing first'.format(car.id))
        direc = [[car.x/SCALE_FACTOR_PLAN, car.y/SCALE_FACTOR_PLAN, -np.rad2deg(car.yaw)]]
        direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*5*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*5*np.sin(car.yaw), -np.rad2deg(car.yaw)])
        direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*10*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*10*np.sin(car.yaw), -np.rad2deg(car.yaw)])
        directive = [np.array(direc)]
        Res_Area = Game.reserve(directive,car)
        self.reserved_areas.update({car.name: Res_Area})
        print('Planner - sending Directive to {0}'.format(car.name))
        car.reverse = True
        await self.out_channels[car.name].send(directive)

    def is_failure_in_acceptable_area(self,gme):
        is_acceptable = gme.is_failure_acceptable(self.obstacles)
        return is_acceptable

    def is_single_failure_in_acceptable_area(self,car,gme):
        obs = {car.name : [car.x,car.y,car.yaw]}
        is_acceptable = gme.is_failure_acceptable(obs)
        return is_acceptable

    def check_reachability(self,spot):
        if self.reachable[spot]:
            return True
        else:
            return False

    async def run(self,Game, Time): # run in trio nursery
        sys.path.append('../motionplanning')
        with open('planning_graph_lanes.pkl', 'rb') as f:
            self.original_lanes_planning_graph = pickle.load(f)
        with open('planning_graph_free.pkl', 'rb') as f:
            self.original_free_planning_graph = pickle.load(f)
        self.planning_graph = self.original_lanes_planning_graph
        send_response_channel, receive_response_channel = trio.open_memory_channel(25)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.check_supervisor,send_response_channel,Game, Time)
            await trio.sleep(1)
            nursery.start_soon(self.update_car_response,receive_response_channel,Game)
