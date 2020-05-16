from components.boxcomponent import BoxComponent
import trio
import _pickle as pickle
import motionplanning.end_planner as path_planner
from prepare.communication import *
from variables.global_vars import *
from variables.parking_data import parking_spots,parking_spots_original
import math
from ipdb import set_trace as st
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point
from shapely import affinity
#from motionplanning.parking_data import parking_spots

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

    async def send_directive_to_car(self, car, end,Game):
        if end == 'Reverse':
            end = (car.x/SCALE_FACTOR_PLAN+-car.direction*10, car.y/SCALE_FACTOR_PLAN+-car.direction*10, car.yaw, 0)
            print('Car moving out of the way to: '+str(end))
        pos = await self.get_car_position(car)
        start = np.zeros(4)
        start[0] = pos[0]/SCALE_FACTOR_PLAN
        start[1] = pos[1]/SCALE_FACTOR_PLAN
        start[2] = -np.rad2deg(pos[2])
        start[3] = 0
        self.get_current_planning_graph(Game)
        print('Planning - {0} driving from {1} to {2}'.format(car.name,start,end))
        traj, weight = self.get_path(start,end) 
        print('Path weight: '+str(weight))
        if traj:
            print('Planner - sending Directive to {0}'.format(car.name))
            await self.out_channels[car.name].send(traj)
        else:
            print('Planner - No Path found - sending Response to Supervisor')
            resp = ('NoPath', car)
            await self.send_response_to_supervisor(resp)
            #await self.out_channels['Supervisor'].send((resp,car))
        await trio.sleep(1)

    def add_obstacle(self, car,Game):
        self.obstacles.update({car.name: (car.x,car.y,car.yaw)})
        self.update_reachability_matrix(Game)

    def rmv_obstacle(self, car,Game):
        self.obstacles.pop(car.name)
        self.update_reachability_matrix(Game)

    def update_reachability_matrix(self,Game):
        start = (140,55,0,0) # start position on the grid
        self.get_current_planning_graph(Game)
        for i in list(parking_spots.keys()):
            end = self.find_spot_coordinates(i)
            traj = self.get_path(start,end) 
            if traj:
                self.reachable[i]=1
            else:
                self.reachable[i]=0

    def is_in_buffer(self,node_x,node_y,obs,):
        # get car box around x,y
        #print(obs)
        for i in range(0,len(obs)):
            x = obs[i][0]
            y = obs[i][1]
            yaw = obs[i][2]
            box = Polygon([(x-10, y+10),(x-10,y-10),(x+15,y-10),(x+15,y+10),(x-15, y+10)])
            rot_box = affinity.rotate(box, np.rad2deg(yaw), origin = (x,y))
        # assume radius of 10 pixels (gridsize) to delete around obstacle center
            if Point(node_x,node_y).intersects(rot_box):
                return True
        return False

    def get_current_planning_graph(self,Game):
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
                # assume obs not in the lane area
                self.planning_graph_in_use = self.original_lanes_planning_graph
                # check if obstacle is in lanes_box
                for obs_box in obs_boxes:
                    if not obs_box.intersects(self.lanes_box):
                        self.planning_graph_in_use = self.original_free_planning_graph
                # find grid nodes around obstacle
                nodes = self.planning_graph_in_use['graph']._nodes
                del_nodes = [(node) for node in nodes if self.is_in_buffer(node[0],node[1],obs)]
                print('deleting these nodes:')
                print(del_nodes)
                self.planning_graph = path_planner.update_plannning_graph(self.planning_graph_in_use, del_nodes)
            else:
                print('Failure blocks the narrow path')
                self.planning_graph = self.original_lanes_planning_graph
        else:
            self.planning_graph = self.original_lanes_planning_graph

    def get_path(self, start, end): 
        #self.get_current_planning_graph()
        try:
            traj, weight = path_planner.get_mpc_path(start,end,self.planning_graph)
        except:
            st()
        if traj and weight < 600:
            return traj, weight
        else: 
            traj = False
            weight = None
            return traj, weight

    async def update_car_response(self, receive_response_channel,Game):
        async with receive_response_channel:
            async for response in receive_response_channel:
                car = response[0]
                resp = response[1]
                print('Planner - receiving "{0}" response from {1}'.format(resp,car.name))
                if response[1]=='Completed':
                    if self.cars.get(car.name,0)=='Assigned':
                        self.cars.update({car.name: 'Parked'})
                    elif self.cars.get(car.name,0)=='Request':
                        self.cars.pop(car.name)
                    await self.send_response_to_supervisor((resp,car))
                if response[1]=='Failure':
                    self.add_obstacle(car,Game)
                    self.cars.update({car.name: 'Failure'})
                    await self.send_response_to_supervisor((resp,car))
                elif response[1]=='Blocked':
                    if self.is_failure_in_acceptable_area(Game):
                        print('Obstacles are in acceptable area.')
                        if self.cars.get(car.name,0)=='Assigned':
                            for key, val in self.spots.items(): 
                                if val == car.name: 
                                    spot = key
                            end = self.find_spot_coordinates(spot)
                            car.replan = True
                            car.status = 'Replan'
                            print('Blocked car {} receives a new directive'.format(car.name))
                            await self.send_directive_to_car(car, end, Game)
                        elif self.cars.get(car.name,0)=='Request':
                            car.status = 'Replan'
                            await self.send_directive_to_car(car, PICK_UP, Game)
                    else:
                        print('No way to plan around the failure - {0} must wait'.format(car.name))
                elif resp=='RequestReservedArea':
                    await self.send_response_to_supervisor((resp,car))
                elif resp[0]=='Conflict':
                    await self.send_response_to_supervisor((resp,car))
                await trio.sleep(0)
                print(self.cars)
                # await self.send_response_to_supervisor((resp,car))

    async def send_response_to_supervisor(self,resp):
        response = resp[0]
        car = resp[1]
        #print(resp)
        print('Planner - sending "{0} - {1}" response to Supervisor'.format(response,car.name))
        await self.out_channels['Supervisor'].send((car,response))
        await trio.sleep(0)
    
    async def check_supervisor(self,send_response_channel,Game, Time):
        async for directive in self.in_channels['Supervisor']:
            car = directive[0]
            todo = directive[1]
            if todo[0] == 'Park':
                print('Planner - receiving directive from Supervisor to park {0} in Spot {1}'.format(car.name,todo[1]))
                self.cars.update({car.name: 'Assigned'})
                create_unidirectional_channel(self, car, max_buffer_size=np.inf)
                self.nursery.start_soon(car.run,send_response_channel,Game, Time)
                self.spots.update({todo[1]: car.name})
                end = self.find_spot_coordinates(todo[1])
                await self.send_directive_to_car(car, end, Game)
            elif todo == 'Pickup':
                print('Planner -  receiving directive from Supervisor to retrieve {0}'.format(car.name))
                await self.send_directive_to_car(car, PICK_UP,Game)
                self.cars.update({car.name : 'Request'})
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Towed':
                # remove the car
                print('Planner - {0} is being towed'.format(car.name))
                self.cars.pop(car.name)
                self.rmv_obstacle(car,Game)
                self.spots.update({self.spots.get(car.name): 0})
            elif todo == 'Wait':
                print('Planner - {0} has to wait until path clears'.format(car.name))
                await self.send_directive_to_car(car, todo,Game)
            elif todo == 'Reverse':
                print('Planner - {0} has to drive out of the way'.format(car.name))
                #await self.send_directive_to_car(car, todo,Game)
                await self.out_channels[car.name].send('Reverse')
            elif todo == 'Back2spot':
                print('Planner - {0} has to drive back into the spot to make space'.format(car.name))
                car.status = 'Replan'
                car.replan = True
                # for key, val in self.spots.items(): 
                #     if val == car.name: 
                #         spot = key
                # end = self.find_spot_coordinates(spot)
                #await self.send_directive_to_car(car, end,Game)
                await self.out_channels[car.name].send('Back2spot')
            elif todo == 'ReserveReverse':
                Res_Area = Game.reserve_reverse(car)
                self.reserved_areas.update({car.name: Res_Area})

    def is_failure_in_acceptable_area(self,gme):
        is_acceptable = gme.is_failure_acceptable(self.obstacles)
        return is_acceptable

    def check_reachability(self,spot):
        if self.reachable[spot]:
            return True
        else:
            return False

    async def run(self,Game, Time):
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
