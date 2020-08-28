# Automated Valet Parking - Garage Supervisor Component
# Josefine Graebener
# California Institute of Technology
# March, 2020

from prepare.boxcomponent import BoxComponent
from variables.geometries import UPPER_SPOTS, LOWER_SPOTS, MIDDLE_BOX
import trio
import random
from variables.global_vars import MAX_NO_PARKING_SPOTS, TOW_TIME, DELAY_THRESH, start_walk_lane, end_walk_lane, start_walk_lane_2, end_walk_lane_2, SCALE_FACTOR_PLAN as SFP
from environment.pedestrian import Pedestrian
from components.planner import Planner
from variables.parking_data import parking_spots_gazebo_test as parking_spots #bad_parking_spot as parking_spots
from ipdb import set_trace as st
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Point, Polygon

class Supervisor(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = dict()
        self.nursery = nursery
        self.spot_no = len(list(parking_spots.keys()))
        self.parking_spots = dict([(i, ("Vacant", "None")) for i in list(parking_spots.keys())])
        self.failures = dict()
        self.priority = dict()
        self.conflicts = []
        self.counter = 1
        self.upper_spots = UPPER_SPOTS
        self.lower_spots = LOWER_SPOTS 
        self.middle_box = MIDDLE_BOX
        self.Logger = None

    async def send_directive_to_planner(self, car,ref):
        directive = [car,ref]
        self.Logger.info('SUPERVISOR - sending {0} - {1} to Planner'.format(car.name,directive[1]))
        await self.out_channels['Planner'].send(directive)

    async def update_parking_spots(self,car):
        for spot, status in self.parking_spots.items():
                if (status[1] == car.name) and not car.status == 'Removed':
                    if (status[0] == 'Assigned') and not car.replan:
                        self.parking_spots[spot]=(('Occupied',car.name))
                        self.cars.update({car.name: 'Parked'})
                        car.parked = True
                        car.new_spot = False
                    elif car.is_at_pickup:
                        self.Logger.info('SUPERVISOR - Car {0} is picked up'.format(car.name))
                        car.requested = False
                        car.picked_up = True
                        car.ref = []
                        self.parking_spots[spot]=(('Vacant','None'))
                        await self.out_channels['GameExit'].send(car)
                        self.spot_no=self.spot_no+1
                        self.cars.pop(car.name)
                        self.priority.pop(car.name)
                        car.status = 'Removed'
                        car.cancel = True
                    elif not car.is_at_pickup and self.cars.get(car.name)=='Requested':
                        await self.send_directive_to_planner(car, 'Pickup')

    async def reserve_reverse_area(self, car): # reserve area for unparking
        await self.send_directive_to_planner(car, 'ReserveReverse')

    async def update_planner_response(self,Planner, Simulation): # process responses from planner
        async with self.in_channels['Planner']:
            async for response in self.in_channels['Planner']:
                car = response[0]
                resp = response[1]
                self.Logger.info('SUPERVISOR - receiving "{0} - {1}" Response from Planner'.format(car.name,resp))
                if resp == 'Completed':
                    await self.update_parking_spots(car)
                elif resp == 'Failure':
                    self.failures.update({car.name: (car.x,car.y,car.yaw)})
                    await self.out_channels['Failure'].send(car)
                    self.cars.update({car.name: 'Failure'})
                    self.Logger.info('SUPERVISOR - initiating towing of {0}'.format(car.name))
                    await self.initiate_towing(car, Simulation)
                elif resp == 'NoPath':
                    if self.cars.get(car.name)== 'Assigned':
                        spot = self.pick_spot(car,Planner, Simulation)
                        for key, value in self.parking_spots.items(): 
                            if value == ('Assigned',car.name): 
                                val = key
                        self.parking_spots[val]=(('Vacant','None'))
                        Simulation.spots.pop(val)
                        self.parking_spots[spot]=(('Assigned',car.name))
                        self.cars.update({car.name: 'Assigned'})
                        Simulation.spots.update({spot: car.id})
                        await self.send_directive_to_planner(car,('Park',spot))
                    elif self.cars.get(car.name)=='Requested':
                        #await self.send_directive_to_planner(car,('Wait'))
                        print('Wait until resolved')
                elif resp == 'RequestReservedArea':
                    #car.reserved = True
                    if car.status != 'Failure':
                        self.Logger.info('SUPERVISOR - Car ID {0} has delay: {1} - Reserving Area'.format(car.id,car.delay))
                        await self.reserve_reverse_area(car)
                elif resp[0] == 'SpotUnreachable':
                    obs = resp[1]
                    for key, value in self.parking_spots.items(): 
                        if value == ('Assigned',car.name): 
                            oldspot = key
                    newspot = self.pick_new_spot(car,obs,Planner)
                    if newspot: # if there exits a possible new spot send updated command
                        car.new_spot = True
                        car.status = 'Replan'
                        Simulation.spots.update({newspot: car.id})
                        self.parking_spots[newspot]=(('Assigned',car.name))
                        Simulation.spots.pop(oldspot)
                        self.parking_spots[oldspot]=(('Vacant','None'))
                        await self.send_directive_to_planner(car,('Park',newspot))
                    else: # drive to old spot
                        car.replan = False
                        car.new_spot = False
                        await self.send_directive_to_planner(car,('Park',oldspot))
                elif resp[0] == 'Conflict':
                    car_list = resp[1]
                    for cars in car_list:
                        #prio = self.priority.get(cars.name)
                        if car.unparking and not cars.unparking:
                            print("{0} delay: {1}".format(car.name,car.delay))
                            if car.delay <= DELAY_THRESH:
                                print('Opposed car has priority ID {0}'.format(cars.id))
                                await self.send_directive_to_planner(car,('Back2spot'))
                                self.conflicts.append(car)
                            else:
                                self.Logger.info('SUPERVISOR - Opposed car has priority {0} - Delay high, Reserve Unparking Area Next'.format(cars.id))
                                await self.send_directive_to_planner(car,('Back2spot'))
                                await self.reserve_reverse_area(car)
                        elif cars.unparking and not car.unparking:
                            print('{0} has priority, ID {1}'.format(car.name,car.id))
                        else:
                            print('Conflict resolving by ID')
                            if id(car)>id(cars):
                                print('{0} has priority, ID {1}'.format(car.name,car.id))
                            else:
                                print('Opposed car has priority {0}'.format(cars.id))
                                if car.unparking:
                                    await self.send_directive_to_planner(car,('Back2spot'))
                                    self.conflicts.append(car)
                                else:
                                    print('Conflict resoling needed at supervisor level')
                                    #await self.send_directive_to_planner(car,('Reverse'))
                await trio.sleep(0)
    
    async def initiate_towing(self,car, Simulation): # tow the failed car
        self.Logger.info('SUPERVISOR - sending towing request')
        await self.out_channels['TowTruck'].send(car)

    async def update_towed_cars(self,Simulation):
        async with self.in_channels['TowTruck']:
            async for car in self.in_channels['TowTruck']:
                directive = [car, 'Towed'] 
                await self.out_channels['Planner'].send(directive)
                self.cars.pop(car.name)
                self.priority.pop(car.name)
                self.failures.pop(car.name)
                await self.out_channels['GameExit'].send(car)
                for spot, value in self.parking_spots.items(): 
                    if value == ('Assigned',car.name) or value == ('Occupied', car.name) or value == ('Requested', car.name): 
                        val = spot
                self.parking_spots[val]=(('Vacant','None'))
                if val in Simulation.spots:
                    Simulation.spots.pop(val)
                self.spot_no=self.spot_no+1
                self.Logger.info('SUPERVISOR - '+str(self.spot_no)+' parking Spots vacant')

    def pick_spot(self,car,pln,sim): # pick a parking spot randomly
        random_list = random.sample(list(self.parking_spots.keys()),len(list(self.parking_spots.keys())))
        for spot in random_list: 
            if (self.parking_spots[spot]==('Vacant','None')) and pln.check_reachability(spot):
                return spot
        return None

    def pick_new_spot(self,car,obs, pln): # strategy to pick a different parking spot, if spot becomes unreachable
        #st()
        # is another spot reachable before the failure
        buffer = 1.0 # m
        buffer_back = 0.0
        buffer_front = buffer
        ordered_dict = dict()
        obs_x = obs.x*SFP
        obs_y = obs.y*SFP
        obs = Point([(obs_x,obs_y)])
        car_loc = Point([(car.x,car.y)])
        car_x = car.x
        print('Obs:'+str(obs_x)+','+str(obs_y))
        if obs.intersects(self.upper_spots) or obs.intersects(self.middle_box): # if failure is in upper row
            #print('Failure in upper row')
            for key,val in parking_spots.items():
                loc_x = val[0]*SFP
                loc_y = val[1]*SFP
                #print(str(key)+': '+str(loc_x)+','+str(loc_y))
                loc = Point([(loc_x,loc_y)])
                if loc.intersects(self.upper_spots): # if spot in upper row
                    #print('Check if between car and failure')
                    if loc_x > (obs_x + buffer_back) and loc_x < (car_x - buffer_front): # if between car and failure
                        #print('Adding spot {0} to list'.format(key))
                        ordered_dict.update({key: val})
            #print('Possible spots:')
            #for key,val in ordered_dict.items():
            #    print(key)
        elif obs.intersects(self.lower_spots):
            #print('Failure in lower row')
            if car_loc.intersects(self.lower_spots): # if car in lower row only pick spots in lower row
                for key,val in parking_spots.items():
                    loc_x = val[0]*SFP
                    loc_y = val[1]*SFP
                    loc = Point([(loc_x,loc_y)])
                    #print(str(key)+': '+str(loc_x)+','+str(loc_y))
                    if loc.intersects(self.lower_spots):
                        if loc_x < (obs_x - buffer_back) and loc_x > (car_x + buffer_front): # if between car and failure
                            #print('Adding spot {0} to list'.format(key))
                            ordered_dict.update({key: val})
            else:
                for key,val in parking_spots.items(): # if car in upper row, can pick spots in upper row and lower row
                    loc_x = val[0]*SFP
                    loc_y = val[1]*SFP
                    loc = Point([(loc_x,loc_y)])
                    #print(str(key)+': '+str(loc_x)+','+str(loc_y))
                    if loc.intersects(self.lower_spots):
                        if loc_x < (obs_x - buffer_back):
                            #print('Adding spot {0} to list'.format(key))
                            ordered_dict.update({key: val})
                    else:
                        if loc_x < (car_x - buffer_front):
                            #print('Adding spot {0} to list'.format(key))
                            ordered_dict.update({key: val})
            #print('Possible spots:')
            #for key,val in ordered_dict.items():
            #    print(key)
        # ordered dict contains all possible parking spots
        # delete the occupied ones and unreachable ones
        #st()
        del_keys = []
        for key,val in ordered_dict.items():
            if not (self.parking_spots[key]==('Vacant','None')):
                del_keys.append(key)
            elif not pln.check_reachable_from_car(key,car):
                del_keys.append(key)
        for key in del_keys:
            ordered_dict.pop(key)
        print('Possible vacant and reachable spots:')
        for key,val in ordered_dict.items():
            print(key)
        #st()
        # pick one of the remaining ones
        if len(ordered_dict)!=0:
            chosen_spot = random.sample(list(ordered_dict.keys()),1)
            chosen_spot = chosen_spot[0]
            print('Picking spot {0} for Car {1}'.format(chosen_spot,car.id))
            return chosen_spot
        else:
            print('No other spot possible')
            return None

    async def process_queue(self,Planner, Simulation): # process arriving customers
        async for car in self.in_channels['Customer']:
            accept_condition = False
            self.Logger.info('SUPERVISOR - '+str(self.spot_no)+' parking Spots vacant')
            print(self.parking_spots)
            #await self.start_random_ped()
            spot = self.pick_spot(car,Planner, Simulation)
            if spot is not None:
                accept_condition = True
            if accept_condition:
                self.Logger.info('SUPERVISOR - {} has been accepted!'.format(car.name))
                car.id = self.counter
                self.counter=self.counter+1
                Simulation.spots.update({spot: car.id})
                await self.out_channels['Customer'].send(True)
                self.spot_no=self.spot_no-1
                await self.out_channels['GameEnter'].send(car)
                self.parking_spots[spot]=(('Assigned',car.name))
                self.cars.update({car.name: 'Assigned'})
                self.priority.update({car.name: '0'})
                await self.send_directive_to_planner(car,('Park',spot))
                #ped = Pedestrian(pedestrian_type=random.choice(['1','2','3','4','5','6']))
                #self.nursery.start_soon(ped.run,start_walk_lane,end_walk_lane)
                #await self.out_channels['GameEnterPeds'].send(ped)
            else:
                await self.out_channels['Customer'].send(False)
                self.Logger.info('SUPERVISOR - Garage fully occupied - A car has been rejected!')

    async def request_queue(self, Simulation): # process customer retrieval requests
        async for car in self.in_channels['Request']:
            self.priority.update({car.name: '1'})
            self.cars.update({car.name: 'Requested'})
            car.retrieving = True
            for key, value in self.parking_spots.items(): 
                if value == ('Assigned',car.name) or value == ('Occupied',car.name): 
                    val = key
            self.parking_spots[val]=(('Requested',car.name))
            Simulation.spots.pop(val)
            self.Logger.info('SUPERVISOR - sending Directive to Planner to retrieve {}'.format(car.name))
            await self.send_directive_to_planner(car, 'Pickup')

    async def start_random_ped(self): # randomly start a pedestrian on the lower crosswalk
        accept_condition = False
        chance = random.randint(0,100)
        if chance <=25:
            accept_condition = True
        if accept_condition:
            ped = Pedestrian(pedestrian_type=random.choice(['1','2','3','4','5','6']))
            self.nursery.start_soon(ped.run,start_walk_lane_2,end_walk_lane_2)
            await self.out_channels['GameEnterPeds'].send(ped)
            self.Logger.info('SUPERVISOR - Adding random pedestrian to lower crosswalk')


    async def run(self, Planner, Time, Simulation, Logger):
        self.Logger = Logger
        self.Logger.info('SUPERVISOR - started')
        #print(self.parking_spots)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.process_queue, Planner, Simulation)
            nursery.start_soon(self.request_queue, Simulation)
            nursery.start_soon(self.update_planner_response,Planner, Simulation)
            nursery.start_soon(self.update_towed_cars,Simulation)