from components.boxcomponent import BoxComponent
import trio
import random
from variables.global_vars import *
from components.pedestrian import Pedestrian
from components.planner import Planner

class Supervisor(BoxComponent):
    def __init__(self, nursery):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = dict()
        self.nursery = nursery
        self.spot_no = MAX_NO_PARKING_SPOTS
        self.parking_spots = dict([ (i, ("Vacant", "None")) for i in range(0,self.spot_no) ])
        self.failures = dict()
        self.priority = dict()

    async def send_directive_to_planner(self, car,ref):
        directive = [car,ref]
        await self.out_channels['Planner'].send(directive)

    async def update_parking_spots(self,car):
        for spot, status in self.parking_spots.items():
                if (status[1] == car.name):
                    if (status[0] == 'Assigned'):
                        self.parking_spots[spot]=(('Occupied',car.name))
                        self.cars.update({car.name: 'Parked'})
                    elif (status[0] == 'Occupied'):
                        self.parking_spots[spot]=(('Vacant','None'))
                        await self.out_channels['GameExit'].send(car)
                        self.spot_no=self.spot_no+1
                        self.cars.pop(car.name)
                        self.priority.pop(car.name)

    async def update_planner_response(self,Planner):
        async with self.in_channels['Planner']:
            async for response in self.in_channels['Planner']:
                car = response[0]
                resp = response[1]
                print('Supervisor - receiving "{0} - {1}" Response from Planner'.format(car.name,resp))
                if resp == 'Completed':
                    await self.update_parking_spots(car)
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
                        await self.send_directive_to_planner(car,('Wait'))
                elif resp[0] == 'Conflict':
                    print(self.priority)
                    prio_1 = self.priority.get(car.name)
                    car_list = resp[1]
                    for cars in car_list:
                        prio = self.priority.get(cars.name)
                        if prio_2>prio_1:
                            print('Opposed car has priority')
                        elif prio_1>prio_2:
                            print('Car has priority')
                        elif car_2.delay>car.delay:
                            print('Opposed car has priority')
                        elif car.delay>car_2.delay:
                            print('Car has priority')
                        else:
                            print('Conflict resolving by ID')
                            if id(car)>id(car_2):
                                print('Car has priority')
                            else:
                                print('Opposed car has priority')
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
        for spot, status in self.parking_spots.items():
            if (status[0]=='Vacant') and pln.check_reachability(spot):
                # self.parking_spots[spot]=(('Assigned',car.name))
                return spot

    async def process_queue(self,Planner):
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
            print('Supervisor - sending Directive to Planner to retrieve {}'.format(car.name))
            await self.send_directive_to_planner(car, 'Pickup')


    async def run(self, Planner):
        print(self.parking_spots)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.process_queue, Planner)
            nursery.start_soon(self.request_queue)
            nursery.start_soon(self.update_planner_response,Planner)