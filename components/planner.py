# Automated Valet Parking - Garage Planner Component
# Josefine Graebener
# California Institute of Technology
# March, 2020

from prepare.boxcomponent import BoxComponent
from variables.geometries import LANES_BOX
import trio
import numpy as np
import _pickle as pickle
import motionplanning.end_planner as path_planner
from prepare.communication import create_bidirectional_channel, create_unidirectional_channel, get_current_time
from variables.global_vars import SCALE_FACTOR_PLAN, PICK_UP, DROP_OFF_PIX, TESTING_MODE
from variables.parking_data import parking_spots_original as parking_spots#,parking_spots_original
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
        self.original_lanes_planning_graph = [] # Planning graph with lane separation
        self.original_free_planning_graph = [] # Planning graph without lane separation
        # self.planning_graph_in_use = [] # Storing planning graph when updating
        # self.planning_graph_park = [] # Planning graph for way to parking spot to ensure loop
        self.planning_graph_reachability = [] # Planning graph for reachability analysis (ensuring loop in lot)
        self.planning_graph_all = [] # Planning graph containing all the failures for reachability analysis and the way to the parking spot
        self.lanes_box = LANES_BOX
        self.reachable = np.ones((max(list(parking_spots.keys()))+1,1)) # if spots can be reached from drop_off, currently all reachable
        self.reserved_areas = dict()
        self.Logger = None
        self.static_obstacle_map = dict()

    async def get_car_position(self,car):
        self.Logger.info('PLANNER - Sending position request to Map system')
        await self.out_channels['Map'].send(car)
        car, pos = await self.in_channels['Map'].receive()
        return pos

    def find_spot_coordinates(self, spot):
        return parking_spots[spot]

    async def send_directive_to_car(self, car, end, Game, original, obstacle): # directive/response system
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
            traj, weight = self.get_path(start,end)
        self.Logger.info('PLANNER - {0} driving from {1} to {2}, Path weight: {3}'.format(car.name,start,end,weight))
        if traj:
            if car.replan and not car.new_spot:
                self.Logger.info('PLANNER - Reserving Replanning Area for Car {0}!!!'.format(car.id))
                if obstacle:
                    Game.reserve_replanning(car, traj, obstacle)
            if not car.replan:
                if Game.check_and_reserve_other_lane(car,traj):
                    Game.reserve_replanning(car, traj, obstacle)
                    car.hold = True
            self.Logger.info('PLANNER - sending Directive to {0}'.format(car.name))
            await self.out_channels[car.name].send(traj)
        else:
            self.Logger.info('PLANNER - No Path found - sending Response to Supervisor')
            resp = ('NoPath', car)
            await self.send_response_to_supervisor(resp)
            #await self.out_channels[car.name].send('OriginalPath')
        await trio.sleep(1)

    async def add_obstacle(self, car,Game): # add failed car coordinates to obstacle list
        self.obstacles.update({car.name: (car.x,car.y,car.yaw)})
        self.update_reachability_matrix(Game)
        await self.check_obscar_on_path(car, Game)

    def rmv_obstacle(self, car,Game): # remove towed car from obstacle list
        self.obstacles.pop(car.name)
        self.update_reachability_matrix(Game)

    def initialize_static_obstacle_map(self,Game):
        for obskey,val in self.static_obstacle_map.items():
            val = [val[0]*SCALE_FACTOR_PLAN,val[1]*SCALE_FACTOR_PLAN,val[2],val[3]*SCALE_FACTOR_PLAN]
            self.obstacles.update({obskey: (val)})
        Game.update_obstacles(self.obstacles)
        self.update_reachability_matrix(Game)

    async def update_obstacle_map(self,Game,Obstacles,Simulation):
        self.obstacles.clear()
        for obskey,val in Obstacles.obs.items():
            val = [val[0]*SCALE_FACTOR_PLAN,val[1]*SCALE_FACTOR_PLAN,val[2],val[3]]
            self.obstacles.update({obskey: (val)})
            await self.check_obs_on_path(val, Game)
        Game.update_obstacles(self.obstacles)
        # await self.check_obs_on_path(obstacle, Game)
        self.update_reachability_matrix(Game)
        Simulation.update_obs_in_sim(self.obstacles,Obstacles)

    def update_reachability_matrix(self,Game):
        # check which parking spots are still reachable and update self.reachable
        # using planning graph which contains all obstacles on the grid
        start = DROP_OFF_PIX # start position on the grid
        self.get_current_planning_graph(Game)
        for i in list(parking_spots.keys()):
            end = self.find_spot_coordinates(i)
            try:
                traj, weight = path_planner.get_mpc_path(start,end,self.planning_graph_all)
            except:
                traj = None
                weight = None
            if traj and not weight == np.inf:
                self.reachable[i]=1
            else:
                self.reachable[i]=0

    async def check_obs_on_path(self,obs,gme):
        self.Logger.info('PLANNER - Checking if added obstacle is on path')
        for carname in list(self.cars):
            for car in gme.cars:
                if carname == car.name:
                    if len(car.ref)!=0:
                        ref = car.ref
                        ref = ref[car.idx:]
                        try:
                            ref = np.concatenate(ref, axis=0)
                        except:
                            st()
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
                            obstacle_plan = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)]) # in grid pixels for planning
                            obs = {obscar.name : [obscar.x,obscar.y,obscar.yaw]} # in meters
                            if self.is_single_failure_in_acceptable_area(obs, gme):
                                self.Logger.info('PLANNER - OBSTACLE ON CAR ID {0} PATH - NEED NEW PATH'.format(car.id))
                                car.status == 'Replan'
                                await self.replan_path(car, gme, obstacle_plan)
                            else:
                                if not car.requested:
                                    # stop car here and replan
                                    resp = (('SpotUnreachable',obstacle_plan), car)
                                    self.Logger.info('PLANNER - Spot for Car {0} became unreachable'.format(car.id))
                                    car.replan = True
                                    car.status = 'Replan'
                                    #st()
                                    await self.send_response_to_supervisor(resp) # to pick other spot

    async def check_obscar_on_path(self,obscar, gme): # check if the failure is on the path of a car in the garage using the Game
        obstacle = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)]).buffer(1.0) # future use car_box
        self.Logger.info('PLANNER - Checking if obstacle is on path')
        for carname in list(self.cars):
            if carname != obscar.name:
                for car in gme.cars:
                    if carname == car.name:
                        if len(car.ref)!=0:
                            ref = car.ref
                            ref = ref[car.idx:]
                            try:
                                ref = np.concatenate(ref, axis=0)
                            except:
                                st()
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
                                obstacle_plan = Point([(obscar.x/SCALE_FACTOR_PLAN,obscar.y/SCALE_FACTOR_PLAN)]) # in grid pixels for planning
                                obs = {obscar.name : [obscar.x,obscar.y,obscar.yaw]} # in meters
                                if self.is_single_failure_in_acceptable_area(obs, gme):
                                    self.Logger.info('PLANNER - FAILURE ON CAR ID {0} PATH - NEED NEW PATH'.format(car.id))
                                    car.status == 'Replan'
                                    await self.replan_path(car, gme, obstacle_plan)
                                else:
                                    if not car.requested:
                                        # stop car here and replan
                                        resp = (('SpotUnreachable',obstacle_plan), car)
                                        self.Logger.info('PLANNER - Spot for Car {0} became unreachable'.format(car.id))
                                        car.replan = True
                                        car.status = 'Replan'
                                        #st()
                                        await self.send_response_to_supervisor(resp) # to pick other spot

    async def replan_path(self, car, Game, obstacle):
        car.status = 'Replan'
        if self.cars.get(car.name,0)=='Assigned':
            for key, val in self.spots.items():
                if val == car.name:
                    spot = key
            end = self.find_spot_coordinates(spot)
            car.replan = True
            self.Logger.info('PLANNER - sends a new directive to Car {}'.format(car.id))
            await self.send_directive_to_car(car, end, Game, False, obstacle)
        elif self.cars.get(car.name,0)=='Request':
            await self.send_directive_to_car(car, PICK_UP, Game, False, obstacle)

    def is_in_buffer(self,node_x,node_y,obs,): # find nodes in a buffer area around the obstacle to delete from grid
        # get car box around x,y
        buffer_back = 6
        buffer_side = 5
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
            self.Logger.info('PLANNER - Updating Planning Graph')
            obs = []
            obs_boxes = []
            obs_all = []
            obs_boxes_all = []
            for key,value in self.obstacles.items():
                obstacle = {key : [value[0],value[1],value[2]]}
                try:
                    buff = value[3]
                except:
                    buff = 15.0
                if self.is_single_failure_in_acceptable_area(obstacle, Game):
                    #print('Failure is in acceptable area')
                    obs.append(value)
                    obs_boxes.append(Point(value[0]/SCALE_FACTOR_PLAN,value[1]/SCALE_FACTOR_PLAN).buffer(buff))
                obs_all.append(value)
                obs_boxes_all.append(Point(value[0]/SCALE_FACTOR_PLAN,value[1]/SCALE_FACTOR_PLAN).buffer(buff))
            obs = [(row[0]/SCALE_FACTOR_PLAN,row[1]/SCALE_FACTOR_PLAN, row[2]) for row in obs]
            obs_all = [(row[0]/SCALE_FACTOR_PLAN,row[1]/SCALE_FACTOR_PLAN, row[2]) for row in obs_all]
            #print(obs)
            #print(obs_all)
            # assume obs not in the acceptable area to use graph with lanes
            planning_graph_in_use = self.original_lanes_planning_graph
            planning_graph_in_use_park = self.planning_graph_reachability
            # check if obstacle is in lanes_box (acceptable area)
            for obs_box in obs_boxes:
                if obs_box.intersects(self.lanes_box):
                    planning_graph_in_use = self.original_free_planning_graph
            # find grid nodes around obstacle
            nodes = planning_graph_in_use['graph']._nodes
            del_nodes = [(node) for node in nodes if self.is_in_buffer(node[0],node[1],obs)]
            DEL_XY_NODES = []
            for del_node in del_nodes:
                if (del_node[0], del_node[1]) not in DEL_XY_NODES:
                    DEL_XY_NODES.append((del_node[0], del_node[1]))
            del_nodes_all = [(node) for node in nodes if self.is_in_buffer(node[0],node[1],obs_all)]
            DEL_XY_NODES_all = []
            for del_node_all in del_nodes_all:
                if (del_node_all[0], del_node_all[1]) not in DEL_XY_NODES_all:
                    DEL_XY_NODES_all.append((del_node_all[0], del_node_all[1]))
            # print('del_xy_nodes:')
            # print(DEL_XY_NODES)
            # print(DEL_XY_NODES_all)
            self.planning_graph = path_planner.update_planning_graph(planning_graph_in_use, DEL_XY_NODES)
            self.planning_graph_all = path_planner.update_planning_graph(self.planning_graph_reachability, DEL_XY_NODES_all)
        else:
            self.planning_graph = self.original_lanes_planning_graph
            self.planning_graph_all = self.planning_graph_reachability

    def get_path(self, start, end): # compute path using grid_planner + end_planner
        check_block = False
        # if end in parking spot
        if end in parking_spots.values():
            planning_graph = self.planning_graph_all
            check_block = True
        else:
            planning_graph = self.planning_graph
        # use planning graph all, if that's not possible use planning_graph
        traj = None
        try:
            traj, weight = path_planner.get_mpc_path(start,end,planning_graph)
        except:
            if check_block:
                try:
                    traj, weight = path_planner.get_mpc_path(start,end,self.planning_graph)
                except:
                    traj = None
                    weight = None
            else:
                traj = None
                weight = None
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
                self.Logger.info('PLANNER - receiving "{0}" response from {1}'.format(resp,car.name))
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
                        await self.send_response_to_supervisor((resp,car))
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
                    obscar = resp[1]
                    obs = obscar
                    if self.is_failure_in_acceptable_area(Game,obs):
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
                            self.Logger.info('PLANNER - Blocked Car {0} needs a new directive'.format(car.id))
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
                        self.Logger.info('PLANNER - No way to plan around the failure - {0} must wait'.format(car.name))
                elif resp=='RequestArea':
                    Game.reserve(None,car)
                    #await self.send_response_to_supervisor((resp,car))
                elif resp=='RequestReservedArea':
                    if car.status != 'Failure':
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
        if TESTING_MODE: # send to TestSuite instead
            self.Logger.info('PLANNER - sending "{0} - {1}" response to TestSuite'.format(response,car.name))
            await self.out_channels['TestSuite'].send((car,response))
        else: # send to AVP supervisor
            self.Logger.info('PLANNER - sending "{0} - {1}" response to Supervisor'.format(response,car.name))
            await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)


    async def check_supervisor(self,send_response_channel,Game, Time): # directive/response system (receiving directives from supervisor)
        if TESTING_MODE:
            in_channel = self.in_channels['TestSuite']
        else:
            in_channel = self.in_channels['Supervisor']
        async for directive in in_channel:
            car = directive[0]
            todo = directive[1]
            print(self.spots)
            if todo[0] == 'Park':
                self.Logger.info('PLANNER - receiving directive from Supervisor to park {0} in Spot {1}'.format(car.name,todo[1]))
                if self.cars.get(car.name,0)==0:
                    create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                    self.nursery.start_soon(car.run,send_response_channel,Game, Time,self.Logger)
                self.cars.update({car.name: 'Assigned'})
                # create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                # self.nursery.start_soon(car.run,send_response_channel,Game, Time,self.Logger)
                delidx = None
                for key,val in self.spots.items():
                    if val == car.name:
                        delidx = key
                if delidx:
                    self.spots.pop(key)
                self.spots.update({todo[1]: car.name})
                end = self.find_spot_coordinates(todo[1])
                await self.send_directive_to_car(car, end, Game, False, None)
            elif todo == 'Pickup':
                self.Logger.info('PLANNER -  receiving directive from Supervisor to retrieve {0}'.format(car.name))
                car.replan = False
                await self.send_directive_to_car(car, PICK_UP,Game, False, None)
                self.cars.update({car.name : 'Request'})
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Towed': # remove the car
                self.Logger.info('PLANNER - {0} is being towed'.format(car.name))
                self.cars.pop(car.name)
                self.rmv_obstacle(car,Game)
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Wait':
                self.Logger.info('PLANNER - {0} has to wait until path clears'.format(car.name))
                await self.send_directive_to_car(car, todo,Game, False, None)
            elif todo == 'Reverse': # not used currently
                self.Logger.info('PLANNER - {0} has to drive out of the way'.format(car.name))
                #await self.out_channels[car.name].send('Reverse')
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
                self.Logger.info('PLANNER - {0} has to drive back into the spot to make space'.format(car.name))
                car.status = 'Replan'
                car.replan = True
                await self.out_channels[car.name].send('Back2spot')
            elif todo == 'ReserveReverse':
                if car.status != 'Failure':
                    Res_Area = Game.reserve_reverse(car)
                    self.reserved_areas.update({car.name: Res_Area})

    async def send_reverse(self,car, Game):
        self.Logger.info('PLANNER - Car {0} - Reversing first'.format(car.id))
        direc = [[car.x/SCALE_FACTOR_PLAN, car.y/SCALE_FACTOR_PLAN, -np.rad2deg(car.yaw)]]
        direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*5*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*5*np.sin(car.yaw), -np.rad2deg(car.yaw)])
        direc.append([car.x/SCALE_FACTOR_PLAN-car.direction*10*np.cos(car.yaw), car.y/SCALE_FACTOR_PLAN-car.direction*10*np.sin(car.yaw), -np.rad2deg(car.yaw)])
        directive = [np.array(direc)]
        Res_Area = Game.reserve(directive,car)
        self.reserved_areas.update({car.name: Res_Area})
        self.Logger.info('PLANNER - sending Directive to {0} to reverse'.format(car.name))
        car.reverse = True
        car.replan = True
        car.status = 'Replan'
        await self.out_channels[car.name].send(directive)

    def is_failure_in_acceptable_area(self,gme, obscar=None):
        obs = dict()
        if obscar:
            obs.clear()
            obs.update({obscar : (obscar.x,obscar.y)})
            is_acceptable = gme.is_failure_acceptable(obs)
        else:
            is_acceptable = gme.is_failure_acceptable(self.obstacles)
        return is_acceptable

    def is_single_failure_in_acceptable_area(self,obs,gme):
        #obs = {car.name : [car.x,car.y,car.yaw]}
        is_acceptable = gme.is_failure_acceptable(obs)
        return is_acceptable

    def check_reachable_from_car(self,spot,car):
        start = (car.x/SCALE_FACTOR_PLAN, car.y/SCALE_FACTOR_PLAN, np.rad2deg(car.yaw)*(-1),0) # start position on the grid
        end = self.find_spot_coordinates(spot)
        traj, _ = self.get_path(start,end)
        if traj:
            return True
        else:
            return False

    def check_reachability(self,spot):
        if self.reachable[spot]:
            return True
        else:
            return False

    async def run(self,Game, Time, Logger, Obstacles, Simulation): # run in trio nursery
        self.Logger = Logger
        self.Logger.info('PLANNER - started')
        sys.path.append('../motionplanning')
        with open(sys.path[0]+'/../motionplanning/planning_graphs/planning_graph_lanes.pkl', 'rb') as f:
            self.original_lanes_planning_graph = pickle.load(f)
        with open(sys.path[0]+'/../motionplanning/planning_graphs/planning_graph_free.pkl', 'rb') as f:
            self.original_free_planning_graph = pickle.load(f)
        with open(sys.path[0]+'/../motionplanning/planning_graphs/planning_graph_reachability.pkl', 'rb') as f:
            self.planning_graph_reachability = pickle.load(f)
        self.planning_graph = self.original_lanes_planning_graph
        #self.planning_graph_park = self.planning_graph_reachability
        self.planning_graph_all = self.planning_graph_reachability
        await trio.sleep(0)
        # create obstacles only used for testing
        if TESTING_MODE:
            self.static_obstacle_map = Obstacles.create_obstacle_map()
            self.initialize_static_obstacle_map(Game)
            Simulation.add_obs_to_sim(self.static_obstacle_map)
        #
        send_response_channel, receive_response_channel = trio.open_memory_channel(25)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.check_supervisor,send_response_channel,Game, Time)
            await trio.sleep(1)
            nursery.start_soon(self.update_car_response,receive_response_channel,Game)
