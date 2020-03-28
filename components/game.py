from components.boxcomponent import BoxComponent
import trio
import numpy as np
import math
from variables.global_vars import *

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
                print('Game System - Removing car from Game')
                self.cars.remove(car)
                await self.out_channels['Exit'].send(car)
                await self.out_channels['ExitSim'].send(car)

    async def keep_track_influx_peds(self):
        async with self.in_channels['GameEnterPeds']:
            async for pedestrian in self.in_channels['GameEnterPeds']:
                print('Game System - Adding new pedestrian to Game')
                #await self.out_channels['PedEnter'].send(pedestrian) # for Map
                await self.out_channels['PedSimulation'].send(pedestrian)
                self.peds.append(pedestrian)

    async def check_car_path(self,car, direction, last_segment):
        # create cone to check
        if last_segment:
            openangle = 45
            length = 3 # m
        else:
            openangle = 45
            length = 6 # m
        for cars in self.cars:
            if car.name != cars.name:
                dx = cars.x - car.x
                dy = cars.y - car.y
                angle = np.rad2deg(np.arctan2(dy, dx))
                #print('Dist'+str(math.sqrt((dx)**2 + (dy)**2)))
                if (math.sqrt((dx)**2 + (dy)**2)<= length): 
                    print('Other car is close')
                    print('My yaw'+str(np.rad2deg(direction*car.yaw)))
                    print('Angle'+str(direction*angle))
                    if direction == 1:
                        lowb = np.rad2deg(car.yaw)-openangle/2
                        upb = np.rad2deg(car.yaw)+openangle/2
                        print('Check if Angle is between '+str(lowb)+'and '+str(upb))
                        if lowb <= angle <= upb:
                            print('{0} stops because other car is in the path'.format(car.name))
                            return False
                    elif direction == -1:
                        lowb = np.rad2deg(car.yaw+np.pi)-openangle/2
                        upb = np.rad2deg(car.yaw+np.pi)+openangle/2
                        print('Check if Angle is between '+str(lowb)+'and '+str(upb))
                        if lowb <= angle <= upb:
                            print('{0} stops because other car is in the path'.format(car.name))
                            return False
        return True

    async def check_car_conflicts(self, car, direction):
        openangle = 45
        length = 6 # m
        conflict_cars = []
        conflict_with = []
        failed = []
        conflict = False
        # check which cars I have a conflict with
        for cars in self.cars:
            if cars.name != car.name:
                dx = cars.x - car.x
                dy = cars.y - car.y
                angle = np.rad2deg(np.arctan2(-dy, dx))
                if (math.sqrt((dx)**2 + (dy)**2)<= length): 
                    if (np.rad2deg(direction*car.yaw)-openangle/2<=angle<=np.rad2deg(direction*car.yaw)+openangle/2):
                        conflict_cars.append(cars)
                        if cars.status=='Failed':
                            failed.append(cars)
        # check if they have a conflict with me
        for cars in conflict_cars:
            dx = car.x - cars.x
            dy = car.y - cars.y
            angle = np.rad2deg(np.arctan2(-dy, dx))
            if (math.sqrt((dx)**2 + (dy)**2)<= length): 
                    if (np.rad2deg(direction*cars.yaw)-openangle/2<=angle<=np.rad2deg(direction*cars.yaw)+openangle/2):
                        conflict_with.append(cars)
                        conflict = True
        return failed, conflict, conflict_with


    async def dropoff_free(self):
        for cars in self.cars:
            dist = math.sqrt((cars.x-DROP_OFF[0])**2 + (cars.y-DROP_OFF[1])**2)
            if dist<=2:
                return False
        return True

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)