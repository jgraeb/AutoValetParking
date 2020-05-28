from components.boxcomponent import BoxComponent
import trio
import numpy as np
import math
from variables.global_vars import *
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity
from shapely.ops import nearest_points
from variables.parking_data import parking_spots
from ipdb import set_trace as st

class Game(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        #self.lot_status = []
        self.car_boxes = dict()
        self.dropoff_box = Polygon([(40, 18), (40, 15), (47, 18), (47, 15), (40, 18)])
        self.pickup_box = Point(260*SCALE_FACTOR_PLAN, 60*SCALE_FACTOR_PLAN).buffer(1.0)
        self.park_boxes = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(1.0) for i in list(parking_spots.keys())]
        self.park_boxes_area = [Point(parking_spots[i][0]*SCALE_FACTOR_PLAN,parking_spots[i][1]*SCALE_FACTOR_PLAN).buffer(3.0) for i in list(parking_spots.keys())]
        self.accept_box = Polygon([(160*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN), (160*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN), (230*SCALE_FACTOR_PLAN, 130*SCALE_FACTOR_PLAN), (230*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN), (160*SCALE_FACTOR_PLAN, 75*SCALE_FACTOR_PLAN)])
        self.reserved_areas = dict()
        self.reserved_areas_requested = dict()

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
                await self.out_channels['Exit'].send(car)
                await self.out_channels['ExitSim'].send(car)
                print('Game System - Removing {0} from Game'.format(car.name))
                self.cars.remove(car)
                #car.cancel = True

    async def keep_track_influx_peds(self):
        async with self.in_channels['GameEnterPeds']:
            async for pedestrian in self.in_channels['GameEnterPeds']:
                print('Game System - Adding new pedestrian to Game')
                #await self.out_channels['PedEnter'].send(pedestrian) # for Map
                await self.out_channels['PedSimulation'].send(pedestrian)
                self.peds.append(pedestrian)

    def reserve_reverse(self,car):
        area = car.current_segment
        #define points on the current segment
        n = len(area)
        #Points = np.zeros((n,2))
        line = [(entry[0],entry[1]) for entry in area]
        linestring = LineString(line)

        # for i in range(0,n):
        #     Points[i][0] = area[i][0]*SCALE_FACTOR_PLAN
        #     Points[i][1] = area[i][1]*SCALE_FACTOR_PLAN
        # if n == 3:
        #     line=LineString([(Points[0]),(Points[1]),(Points[2])])
        # elif n == 2:
        #     line=LineString([(Points[0]),(Points[1])])
        # elif n == 4:
        #     line=LineString([(Points[0]),(Points[1]),(Points[2]),(Points[3])])
        area = linestring.buffer(3.5)
        print('Reserving for Car ID {0}'.format(car.id))
        print(car.current_segment)
        print(line)
        self.reserved_areas_requested.update({car: area})
        self.update_reserved_areas()
        #car.reserved = True
    
    def update_reserved_areas(self):
        items_to_delete=[]
        skip_line = False
        for key_req,val_req in self.reserved_areas_requested.items():
            accept = True
            if skip_line:
                break
            for key,val in self.reserved_areas.items():
                if val_req.intersects(val):
                    accept = False
                    skip_line = True
            if accept:
                self.reserved_areas.update({key_req: val_req})
                items_to_delete.append(key_req)
                key_req.reserved = True
        for item in items_to_delete:
            self.reserved_areas_requested.pop(item)
        print('Areas Reserved:')
        print(self.reserved_areas)
        for key in self.reserved_areas.keys():
            print(key.id)
        print('Areas Requested:')
        print(self.reserved_areas_requested)
        for key in self.reserved_areas_requested.keys():
            print(key.id)

    def release_reverse_area(self, car):
        print('{0} releasing reserved area'.format(car.id))
        self.reserved_areas.pop(car)
        self.update_reserved_areas()

    def is_car_free_of_reserved_area(self,car):
        if not self.reserved_areas:
            if not self.reserved_areas_requested:
                return True
        area = self.reserved_areas.get(car,0)
        if area == 0:
            area = self.reserved_areas_requested.get(car,0)
        mycone = self.get_vision_cone(car)
        mybox = self.car_boxes.get(car.name,0)
        try:
            if mycone.intersects(area) or mybox.intersects(area):
                return False
        except:
            st()
        return True

    def get_vision_cone(self,car):
        buffer = car.v/10*0.4
        if car.replan and car.last_segment:# and not car.retrieving:
            openangle = 30
            length = 3
        elif car.last_segment and not car.retrieving:
            openangle = 45
            length = 4 # m
        elif car.unparking:
            openangle = 60
            length = 4+buffer # m # changes here to try
        elif car.close or car.replan:
            openangle = 30
            length = 5.0 # m
        else:
            openangle = 30
            length = 6 + buffer # m
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

    def get_vision_cone_pedestrian(self,car):
        length = 6 # m
        cone = Polygon([(car.x,car.y),(car.x,car.y+1),(car.x+length,car.y+1),(car.x+length,car.y-1),(car.x,car.y-1),(car.x,car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def rad2rad(self,angle):
        if abs(angle)>np.pi:
            if angle > np.pi:
                angle = np.mod(angle, 2*np.pi)
                diff = angle - np.pi
                angle = diff - np.pi
            elif angle < -np.pi:
                angle = np.mod(angle, 2*np.pi)
                diff = angle + np.pi
                angle = np.pi - diff
        return angle

    def check_car_path(self,car):
        conflict_cars = []
        failed = False
        blocked = False
        clearpath = True
        conflict = False
        stop_reserved = False
        mycone = self.get_vision_cone(car)
        self.car_boxes.clear()
        for cars in self.cars:
            box = Polygon([(cars.x-0.5,cars.y+1),(cars.x-0.5,cars.y-1),(cars.x+3,cars.y+1),(cars.x+3,cars.y-1)])
            rot_box = affinity.rotate(box, np.rad2deg(cars.yaw), origin = (cars.x,cars.y))
            self.car_boxes.update({cars.name: rot_box})
        # make sure that there are no collisions ever
        mybox = self.car_boxes.get(car.name,0)
        # for key,val in self.car_boxes.items():
        #     if key != car.name:
        #         if mybox.distance(val)<=0.0:
        #             point = nearest_points(mybox, val)
        #             angle = np.arctan2((point[0].y-car.y),(point[0].x-car.x))
        #             print('Angle: '+str(angle))
        #             print('Caryaw'+str(car.yaw))
        #             yaw = self.rad2rad(car.yaw)
        #             angle = self.rad2rad(angle)
        #             print('Angle after conv: '+str(angle))
        #             print('Caryaw'+str(yaw))
        #             if car.direction ==1: 
        #                 if abs(angle-yaw) < np.pi/2 or abs(angle-yaw) > 3/4*np.pi:
        #                     clearpath = False
        #             car_angle = self.rad2rad(yaw+np.pi)
        #             print('Backwards: '+str(car_angle))
        #             #st()
        #             if car.direction ==-1 and abs(angle-car_angle)< np.pi/2:
        #                 clearpath = False
        # Now check if car box and vision cone intersect
        # check for failed and blocked cars
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
        clearpath = True
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    #clearpath = False
                    for cars in self.cars:
                        if cars.name == key:
                            if cars.status=='Failure':
                                #failed_cars.append(cars)
                                failed = True
                                print('{0} blocked by a failed car, ID {1}'.format(car.name, car.id))
                                clearpath = False
                            elif cars.parked:
                                print('Blocked by a parked car - Go on ID {0}'.format(car.id))
                                #clearpath = True
                            else:
                                conflict_cars.append(cars)
                                clearpath = False
        # check if a pedestrian is in the cone and I am not in the pedestrian's way
        mypedcone = self.get_vision_cone_pedestrian(car)
        for ped in self.peds:
            x_m = ped.state[0]/SCALE_FACTOR_SIM
            y_m = ped.state[1]/SCALE_FACTOR_SIM
            ped_point = Polygon([(x_m+2,y_m+1),(x_m+2,y_m-1),(x_m-1,y_m-1), (x_m-1,y_m+1),(x_m+2,y_m+1)])
            if ped_point.intersects(mypedcone) and not car.last_segment and not mybox.intersects(ped_point):
                clearpath = False
                print('{0} stops because a pedestrian is in the path, ID {1}'.format(car.name, car.id))
        # check if they have a conflict with me
        for cars in conflict_cars:
            cone = self.get_vision_cone(cars)
            #print(cone)
            try:
                cone.intersects(mybox)
            except:
                st()
            if not cone.intersects(mybox):
                conflict_cars.remove(cars)
            else:
                conflict = True
        # check before entering reserved areas
        myarea = self.reserved_areas.get(car,0)
        #if not mybox.intersects(myarea): # make sure to leave your area first
        for key,val in self.reserved_areas.items():
            if key != car:
                if val.intersects(mycone) and not val.intersects(mybox):
                    if myarea:
                        if not myarea.intersects(mybox):
                            stop_reserved = True
                            clearpath = False
                            print('BBBBBBBBBB ---- {0} stopped because of reserved area for Car {1} ahead'.format(car.id,key.id))
                    else:
                        stop_reserved = True
                        clearpath = False
                        print('BBBBBBBBBB ---- {0} stopped because of reserved area for Car {1} ahead'.format(car.id,key.id))
                    print('Stopping for reserved area ID {0}'.format(car.id))
        return clearpath, conflict_cars, failed, conflict, blocked, stop_reserved

    def clear_before_unparking(self,car):
        self.update_car_boxes()
        mycone = self.get_vision_cone(car)
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    print('{0} Waiting'.format(car.name))
                    return False
        # check neighbors are not in driving status
        my_area = Point(car.x,car.y).buffer(4.0)
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
        self.update_car_boxes()
        for key,val in self.car_boxes.items():
            if self.dropoff_box.intersects(val):
                #print('DROPOFF OCCUPIED')
                return False
        return True

    def is_car_at_pickup(self,car):
        self.update_car_boxes()
        mybox = self.car_boxes.get(car.name)
        if self.pickup_box.intersects(mybox):
            print('{0} is at pick up, ID {1}'.format(car.name,car.id))
            # print('X '+str(car.x))
            # print('Y '+str(car.y))
            # print('Pickup Box '+str(260*SCALE_FACTOR_PLAN)+str(60*SCALE_FACTOR_PLAN))
            #st()
            return True
        return False

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
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)