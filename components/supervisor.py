from components.boxcomponent import BoxComponent
import trio
import random
from variables.global_vars import *
from components.pedestrian import Pedestrian
from components.planner import Planner
from variables.parking_data import parking_spots #bad_parking_spot as parking_spots
from ipdb import set_trace as st

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
        self.counter = 0

    async def send_directive_to_planner(self, car,ref):
        directive = [car,ref]
        await self.out_channels['Planner'].send(directive)
        print('Supervisor sending {0} - {1} to Planner'.format(car.name,directive))

    async def update_parking_spots(self,car):
        for spot, status in self.parking_spots.items():
                if (status[1] == car.name) and not car.status == 'Removed':
                    # print('AAAAAA - Updating the parking spots')
                    # print(car.replan)
                    # print(car.is_at_pickup)
                    if (status[0] == 'Assigned') and not car.replan:
                        self.parking_spots[spot]=(('Occupied',car.name))
                        self.cars.update({car.name: 'Parked'})
                    elif car.is_at_pickup:
                        print('Supervisor - Car {0} is picked up'.format(car.name))
                        car.requested = False
                        self.parking_spots[spot]=(('Vacant','None'))
                        await self.out_channels['GameExit'].send(car)
                        self.spot_no=self.spot_no+1
                        self.cars.pop(car.name)
                        self.priority.pop(car.name)
                        car.status = 'Removed'
                        car.cancel = True
                    elif not car.is_at_pickup and self.cars.get(car.name)=='Requested':
                        await self.send_directive_to_planner(car, 'Pickup')

    async def reserve_reverse_area(self, car):
        await self.send_directive_to_planner(car, 'ReserveReverse')

    async def update_planner_response(self,Planner):
        async with self.in_channels['Planner']:
            async for response in self.in_channels['Planner']:
                car = response[0]
                resp = response[1]
                print('Supervisor - receiving "{0} - {1}" Response from Planner'.format(car.name,resp))
                if resp == 'Completed':
                    #st()
                    await self.update_parking_spots(car)
                    # if self.cars.get(car.name)=='Requested':
                    #     #print('Supervisor - sending Directive to Planner to retrieve {}'.format(car.name))
                    #     #await self.send_directive_to_planner(car, 'Pickup')
                    #     await self.update_parking_spots(car)
                    # if self.cars.get(car.name)=='Assigned':
                    #     # for key, value in self.parking_spots.items(): 
                    #     #     if value == ('Assigned',car.name): 
                    #     #         spot = key
                    #     # await self.send_directive_to_planner(car,('Park',spot))
                    #     await self.update_parking_spots(car)
                elif resp == 'Failure':
                    self.failures.update({car.name: (car.x,car.y,car.yaw)})
                    await self.out_channels['Failure'].send(car)
                    self.cars.update({car.name: 'Failure'})
                    print('Supervisor initiating towing of {0}'.format(car.name))
                    await self.tow(car)
                elif resp == 'NoPath':
                    if self.cars.get(car.name)== 'Assigned':
                        spot = self.pick_spot(car,Planner)
                        for key, value in self.parking_spots.items(): 
                            if value == ('Assigned',car.name): 
                                val = key
                        self.parking_spots[val]=(('Vacant','None'))
                        self.parking_spots[spot]=(('Assigned',car.name))
                        self.cars.update({car.name: 'Assigned'})
                        await self.send_directive_to_planner(car,('Park',spot))
                    elif self.cars.get(car.name)=='Requested':
                        #await self.send_directive_to_planner(car,('Wait'))
                        print('Wait until resolved')
                elif resp == 'RequestReservedArea':
                    #car.reserved = True
                    print("Car ID {0} has delay: {1} - Reserving Area".format(car.id,car.delay))
                    await self.reserve_reverse_area(car)
                elif resp[0] == 'Conflict':
                    #print(self.priority)
                    prio_car = self.priority.get(car.name)
                    car_list = resp[1]
                    for cars in car_list:
                        prio = self.priority.get(cars.name)
                        if car.unparking and not cars.unparking:
                            print("{0} delay: {1}".format(car.name,car.delay))
                            if car.delay <= DELAY_THRESH:
                                print('Opposed car has priority ID {0}'.format(cars.id))
                                await self.send_directive_to_planner(car,('Back2spot'))
                                self.conflicts.append(car)
                            else:
                                print('Opposed car has priority {0} - Delay high, Reserve Unparking Area Next'.format(cars.id))
                                await self.send_directive_to_planner(car,('Back2spot'))
                                await self.reserve_reverse_area(car)
                                #car.reserved = True
                        elif cars.unparking and not car.unparking:
                            print('{0} has priority, ID {1}'.format(car.name,car.id))
                        # elif prio>prio_1:
                        #     print('Opposed car has priority')
                        #     #await self.send_directive_to_planner(car,('Reverse'))
                        # elif prio_1>prio:
                        #     print('Car has priority')
                        # elif cars.delay>car.delay:
                        #     print('Opposed car has priority')
                        #     #await self.send_directive_to_planner(car,('Reverse'))
                        # elif car.delay>cars.delay:
                        #     print('Car has priority')
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
                                    await self.send_directive_to_planner(car,('Reverse'))
                await trio.sleep(0)
    
    async def tow(self,car):
        await trio.sleep(TOW_TIME)
        # removing car from lot
        await self.out_channels['GameExit'].send(car)
        directive = [car, 'Towed'] 
        await self.out_channels['Planner'].send(directive)
        self.cars.pop(car.name)
        self.priority.pop(car.name)
        self.failures.pop(car.name)
        for spot, value in self.parking_spots.items(): 
            if value == ('Assigned',car.name) or value == ('Occupied', car.name): 
                val = spot
        self.parking_spots[val]=(('Vacant','None'))
        self.spot_no=self.spot_no+1
        print(str(self.spot_no)+' parking Spots vacant')

    def pick_spot(self,car,pln):
        #print(list(self.parking_spots.keys()))
        random_list = random.sample(list(self.parking_spots.keys()),len(list(self.parking_spots.keys())))
        for spot in random_list: 
            if (self.parking_spots[spot]==('Vacant','None')) and pln.check_reachability(spot):
                # self.parking_spots[spot]=(('Assigned',car.name))
                return spot

    async def process_queue(self,Planner):
        async for car in self.in_channels['Customer']:
            accept_condition = False
            print(str(self.spot_no)+' parking Spots vacant')
            print(self.parking_spots)
            await self.start_random_ped()
            if self.spot_no>0:
                accept_condition = True
            if accept_condition:
                print('{} has been accepted!'.format(car.name))
                car.id = self.counter
                self.counter=self.counter+1
                await self.out_channels['Customer'].send(True)
                self.spot_no=self.spot_no-1
                await self.out_channels['GameEnter'].send(car)
                spot = self.pick_spot(car,Planner)
                self.parking_spots[spot]=(('Assigned',car.name))
                self.cars.update({car.name: 'Assigned'})
                self.priority.update({car.name: '0'})
                await self.send_directive_to_planner(car,('Park',spot))
                ped = Pedestrian(pedestrian_type=random.choice(['1','2','3','4','5','6']))
                self.nursery.start_soon(ped.run,start_walk_lane,end_walk_lane)
                await self.out_channels['GameEnterPeds'].send(ped)
            else:
                await self.out_channels['Customer'].send(False)
                print('Garage fully occupied - A car has been rejected!')

    async def request_queue(self):
        async for car in self.in_channels['Request']:
            #while not self.cars.get(car.name, 0)=='Parked':#car.status == 'Driving' or car.status == 'Stop':
            #    await trio.sleep(10)
            self.priority.update({car.name: '1'})
            self.cars.update({car.name: 'Requested'})
            car.retrieving = True
            car.requested = True
            for key, value in self.parking_spots.items(): 
                if value == ('Assigned',car.name) or value == ('Occupied',car.name): 
                    val = key
            self.parking_spots[val]=(('Requested',car.name))
            print('Supervisor - sending Directive to Planner to retrieve {}'.format(car.name))
            await self.send_directive_to_planner(car, 'Pickup')

    async def start_random_ped(self):
        accept_condition = False
        chance = random.randint(0,100)
        if chance <=25:
            accept_condition = True
        if accept_condition:
            ped = Pedestrian(pedestrian_type=random.choice(['1','2','3','4','5','6']))
            self.nursery.start_soon(ped.run,start_walk_lane_2,end_walk_lane_2)
            await self.out_channels['GameEnterPeds'].send(ped)
            print('Adding random pedestrian')


    async def run(self, Planner, Time):
        print(self.parking_spots)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.process_queue, Planner)
            nursery.start_soon(self.request_queue)
            nursery.start_soon(self.update_planner_response,Planner)