from components.boxcomponent import BoxComponent
import trio
import numpy as np
import math
from variables.global_vars import *
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point
from shapely import affinity

class Game(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.lot_status = []
        self.car_boxes = dict()
        self.dropoff_box = Polygon([(40, 18), (40, 15), (47, 18), (47, 15), (40, 18)])

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

    def get_vision_cone(self,car):
        if car.last_segment:
            openangle = 45
            length = 3 # m
        else:
            openangle = 45
            length = 6 # m
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    async def check_car_path(self,car):
        conflict_cars = []
        failed = []
        clear = True
        conflict = False
        mycone = self.get_vision_cone(car)
        self.car_boxes.clear()
        for cars in self.cars:
            box = Polygon([(cars.x-0.5,cars.y+1),(cars.x-0.5,cars.y-1),(cars.x+3,cars.y+1),(cars.x+3,cars.y-1)])
            rot_box = affinity.rotate(box, np.rad2deg(cars.yaw), origin = (cars.x,cars.y))
            self.car_boxes.update({cars.name: rot_box})
        #print(self.car_boxes)
        # Now check if car box and vision cone intersect
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    clear = False
                    print('{0} stops because other car is in the path'.format(car.name))
                    if cars.status=='Failed':
                        failed.append(cars)
                    else:
                        conflict_cars.append(cars)
        # check if a pedestrian is in the cone
        for ped in self.peds:
            x_m = ped.state[0]/SCALE_FACTOR_SIM
            y_m = ped.state[1]/SCALE_FACTOR_SIM
            ped_point = Point(x_m,y_m).buffer(1.0)
            if ped_point.intersects(mycone):
                clear = False
                print('{0} stops because a pedestrian is in the path'.format(car.name))
        # check if they have a conflict with me
        mybox = self.car_boxes.get(car.name,0)
        for cars in conflict_cars:
            cone = self.get_vision_cone(cars)
            if not cone.intersects(mybox):
                conflict_cars.remove(cars)
            else:
                conflict = True
        return clear, conflict_cars, failed

    async def dropoff_free(self):
        # update car boxes
        self.car_boxes.clear()
        for cars in self.cars:
            box = Polygon([(cars.x-0.5,cars.y+1),(cars.x-0.5,cars.y-1),(cars.x+3,cars.y+1),(cars.x+3,cars.y-1)])
            rot_box = affinity.rotate(box, np.rad2deg(cars.yaw), origin = (cars.x,cars.y))
            self.car_boxes.update({cars.name: rot_box})
        for key,val in self.car_boxes.items():
            if self.dropoff_box.intersects(val):
                #print('DROPOFF OCCUPIED')
                return False
        return True

    async def run(self):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)