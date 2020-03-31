from components.boxcomponent import BoxComponent
import trio
import numpy as np
import math
from variables.global_vars import *
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point
from shapely import affinity
from variables.parking_data import parking_spots

class Game(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.lot_status = []
        self.car_boxes = dict()
        self.dropoff_box = Polygon([(40, 18), (40, 15), (47, 18), (47, 15), (40, 18)])
        self.park_boxes = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(1.0) for i in list(parking_spots.keys())]
        self.park_boxes_area = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(3.0) for i in list(parking_spots.keys())]
        self.accept_box = Polygon([(160*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN), (160*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN), (230*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN), (230*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN), (160*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN)])

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
        elif car.unparking:
            openangle = 60
            length = 6 # m
        elif car.close or car.replan:
            openangle = 30
            length = 5.0 # m
        else:
            openangle = 45
            length = 6 # m
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking and not car.last_segment:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def get_vision_cone_blocked(self,car):
        openangle = 45
        length = 8 # m
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def check_car_path(self,car):
        conflict_cars = []
        failed = False
        blocked = False
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
        if not car.last_segment and not car.close:
            myblocked_cone = self.get_vision_cone_blocked(car)
            for key,val in self.car_boxes.items():
                if key != car.name:
                    if myblocked_cone.intersects(val):
                        for cars in self.cars:
                            if cars.name == key:
                                if cars.status=='Failure' or cars.status =='Blocked':
                                    blocked = True 
                                    print('Failure or blocked car ahead')

        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    clear = False
                    print('{0} stops because other car is in the path'.format(car.name))
                    for cars in self.cars:
                        if cars.name == key:
                            if cars.status=='Failure':
                                #failed_cars.append(cars)
                                failed = True
                                print('{0} blocked by a failed car'.format(car.name))
                            else:
                                conflict_cars.append(cars)
        # check if a pedestrian is in the cone
        for ped in self.peds:
            x_m = ped.state[0]/SCALE_FACTOR_SIM
            y_m = ped.state[1]/SCALE_FACTOR_SIM
            ped_point = Point(x_m,y_m).buffer(0.5)
            if ped_point.intersects(mycone):
                clear = False
                print('{0} stops because a pedestrian is in the path'.format(car.name))
        # check if they have a conflict with me
        mybox = self.car_boxes.get(car.name,0)
        for cars in conflict_cars:
            cone = self.get_vision_cone(cars)
            #print(cone)
            if not cone.intersects(mybox):
                conflict_cars.remove(cars)
            else:
                conflict = True
                #print('There is a conflict')
        return clear, conflict_cars, failed, conflict, blocked

    def clear_before_unparking(self,car):
        self.update_car_boxes()
        mycone = self.get_vision_cone(car)
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    print('{0} Waiting'.format(car.name))
                    return False
        # check neighbors are not in driving status
        my_area = Point(car.x,car.y).buffer(3.0)
        for key,val in self.car_boxes.items():
            if key != car.name:
                if my_area.intersects(val):
                    for cars in self.cars:
                        if cars.name == key and cars.status == 'Driving':
                            return False
        return True

    def update_car_boxes(self):
        self.car_boxes.clear()
        for cars in self.cars:
            box = Polygon([(cars.x-0.5,cars.y+1),(cars.x-0.5,cars.y-1),(cars.x+3,cars.y+1),(cars.x+3,cars.y-1)])
            rot_box = affinity.rotate(box, np.rad2deg(cars.yaw), origin = (cars.x,cars.y))
            self.car_boxes.update({cars.name: rot_box})

    def dropoff_free(self):
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

    def is_car_in_spot(self,car):
        #print('Checking if car {0} is in spot:'.format(car.name))
        self.update_car_boxes()
        mycar_box = self.car_boxes.get(car.name)
        for park_box in self.park_boxes:
            if mycar_box.intersects(park_box):
                return True
        return False

    def is_car_close_2_spot(self,car):
        #print('Checking if car {0} is in spot:'.format(car.name))
        self.update_car_boxes()
        mycar_box = self.car_boxes.get(car.name)
        for park_box in self.park_boxes_area:
            if mycar_box.intersects(park_box):
                return True
        return False

    def is_failure_acceptable(self, obsdict):
        for key,val in obsdict.items():
            if not Point(val[0],val[1]).buffer(3).intersects(self.accept_box):
                return False
        return True



    async def run(self):
        #print(parking_spots)
        #print(parking_spots[1][1])
        #print(self.park_boxes)
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)