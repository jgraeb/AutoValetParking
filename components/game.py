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

    async def check_car_path(self,car):
        # create conic region to check
        openangle = 45
        length = 7 # m
        for cars in self.cars:
            if car.name != cars.name:
                dx = cars.x - car.x
                dy = cars.y - car.y
                angle = np.rad2deg(np.arctan2(-dy, dx))
                #print('Dist'+str(math.sqrt((dx)**2 + (dy)**2)))
                if (math.sqrt((dx)**2 + (dy)**2)<= length): 
                    # print('Other car is close')
                    # print('My yaw'+str(car.yaw))
                    # print('Angle'+str(angle))
                    if (np.rad2deg(car.yaw)-openangle/2<=angle<=np.rad2deg(car.yaw)+openangle/2):
                        return False
        return True

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