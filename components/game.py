# Automated Valet Parking - Garage Game Component
# Josefine Graebener
# California Institute of Technology
# March, 2020

from prepare.boxcomponent import BoxComponent
from variables.geometries import DROPOFF_BOX, PICKUP_BOX, PARK_BOXES, PARK_BOXES_AREA, FAILURE_ACCEPT_BOX_1,FAILURE_ACCEPT_BOX_2, LANE_1_BOX, LANE_2_BOX
import trio
import numpy as np
import math
from components.camera.gametools import make_line, remove_duplicates, rad2rad
from variables.global_vars import SCALE_FACTOR_PLAN, SCALE_FACTOR_SIM, TESTING_MODE
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity
from shapely.ops import nearest_points, split
from variables.parking_data import parking_spots_original as parking_spots
from ipdb import set_trace as st
from collections import OrderedDict

class Game(BoxComponent):
    def __init__(self):
        super().__init__()
        self.name = self.__class__.__name__
        self.cars = []
        self.peds = []
        self.car_boxes = dict()
        self.dropoff_box = DROPOFF_BOX
        self.pickup_box = PICKUP_BOX
        self.park_boxes = PARK_BOXES
        self.park_boxes_area = PARK_BOXES_AREA
        self.accept_box = FAILURE_ACCEPT_BOX_1
        self.accept_box_2 = FAILURE_ACCEPT_BOX_2
        self.lane1box = LANE_1_BOX
        self.lane2box = LANE_2_BOX
        self.reserved_areas = dict()
        self.reserved_areas_requested = OrderedDict()
        self.trajs = dict()
        self.Logger = None
        self.obstacles = dict()

    async def keep_track_influx(self):
        if TESTING_MODE:
            in_channel = self.in_channels['TestSuite']
        else:
            in_channel = self.in_channels['GameEnter']
        async with in_channel:
            async for car in in_channel:
                self.Logger.info('GAME - Adding new car to Game')
                await self.out_channels['Enter'].send(car)
                await self.out_channels['Simulation'].send(car)
                self.cars.append(car)

    async def keep_track_outflux(self):
        async with self.in_channels['GameExit']:
            async for car in self.in_channels['GameExit']:
                await self.out_channels['ExitSim'].send(car)
                await self.out_channels['Exit'].send(car)
                #await self.out_channels['ExitSim'].send(car)
                self.Logger.info('GAME - Removing {0} from Game / ID {1}'.format(car.name,car.id))
                self.cars.remove(car)
                #car.cancel = True

    async def keep_track_influx_peds(self):
        async with self.in_channels['GameEnterPeds']:
            async for pedestrian in self.in_channels['GameEnterPeds']:
                self.Logger.info('GAME - Adding new pedestrian to Game')
                #await self.out_channels['PedEnter'].send(pedestrian) # for Map
                await self.out_channels['PedSimulation'].send(pedestrian)
                self.peds.append(pedestrian)

    def reserve(self, ref, car):
        if not ref:
            path = car.ref[car.idx:]
            newlinestring = make_line(path) # compute shapely line in meters
            self.Logger.info('GAME - Reserving for Car ID {0}'.format(car.id))
            res_path = newlinestring.intersection(self.accept_box)
            # reserve new path until getting back to original path
            try:
                n = len(res_path.coords)
            except:
                st()
            if len(res_path.coords)>=1:
                try:
                    traj = res_path[0] # if reserved path is multilinestring
                except:
                    traj = res_path

                try:
                    point = Point([traj.coords[-1]])
                except:
                    st()
                try:
                    traj = split(newlinestring,point)
                except:
                    st()
                traj = traj[0]
                area = traj.buffer(3.5)
                self.reserved_areas_requested.update({car: area})
                self.trajs.update({car: (traj,point)})
                self.update_reserved_areas()
        else:
            #st()
            newlinestring = make_line(ref)
            area = newlinestring.buffer(3.5)
            point = None
            traj = newlinestring
            #if car in self.reserved_areas:
            #    self.reserved_areas.pop(car)
            self.reserved_areas_requested.update({car: area})
            self.trajs.update({car: (traj,point)})
            self.update_reserved_areas()

    def reserve_reverse(self,car):
        area = car.current_segment
        #define points on the current segment
        line = [(entry[0]*SCALE_FACTOR_PLAN,entry[1]*SCALE_FACTOR_PLAN) for entry in area]
        linestring = LineString(line)
        area = linestring.buffer(3.5)
        self.Logger.info('GAME - Reserving for Car ID {0}'.format(car.id))
        print(car.current_segment)
        print(line)
        self.reserved_areas_requested.update({car: area})
        self.update_reserved_areas()

    def check_and_reserve_other_lane(self,car,newtraj):
        # here reserve other lane if necessary
        self.Logger.info('GAME - Checking if Car {0} is crossing into other lane'.format(car.id))
        newlinestring = make_line(newtraj)
        if newlinestring.intersects(self.lane2box) and newlinestring.intersects(self.lane1box):
            self.Logger.info('GAME - Have to reserve other lane for  Car {0}'.format(car.id))
            # make sure car starts on path only if reserved area is free
            return True
        return False

    def is_reserved_area_clear(self,car): # check that area is already reserved and clear of other cars
        if car not in self.reserved_areas:
            return False
        else:
            area = self.reserved_areas.get(car)
            self.update_car_boxes()
            for key,val in self.car_boxes.items():
                if key != car.name:
                    if val.intersects(area):
                        for cars in self.cars:
                            if cars.name == key and cars.status != 'Failure':
                                return False
        return True


    def reserve_replanning(self,car,newtraj,obs): # find area to reserve
        # new trajectory from planner around obstacle
        newlinestring = make_line(newtraj)
        #print(car.ref)
        if len(car.ref)>1 and obs is not None: # if we have an old trajectory stored for the car
            oldpath = car.ref[car.idx:]
            oldlinestring = make_line(oldpath)
            # find location of failure on path
            try:
                failpoint = nearest_points(oldlinestring,obs)
            except:
                st()
            originalpath = split(oldlinestring,failpoint[0])
            originaltraj = originalpath[1]
            # find intersection of old path and new path after passing the failure
            intersection = originaltraj.intersection(newlinestring)
            try:
                point = Point(intersection[0].coords[0])
            except:
                st()
        else: # if the car does not have an old trajectory or was reversing last
            intersection = newlinestring.intersection(self.accept_box)
            print(intersection)
            try:
                point = Point(intersection.coords[-1])
            except:
                try:
                    point = Point(intersection[-1].coords[-1])
                except:
                    st()
        print('Intersection is at: {0}, {1}'.format(point.x,point.y))
        traj = split(newlinestring,point)
        # reserve new path until getting back to original path
        traj = traj[0]
        area = traj.buffer(3.5)
        self.Logger.info('GAME - Reserving Replanning Area for Car ID {0}'.format(car.id))
        self.reserved_areas_requested.update({car: area})
        self.trajs.update({car: (traj,point)})
        self.update_reserved_areas()

    def update_reserved_area_for_car(self,car): # make sure to reserve area on path and release behind car
        if not car.unparking:
            path = car.ref[car.idx:]
            path = make_line(path)
            if car not in self.trajs:
                return
            else:
                _, point = self.trajs.get(car,0)
                if point:
                    if point.intersects(path):
                        segments = split(path,point)
                    if not point.intersects(path) or len(segments)==1: # stop updating the reserved area if car is leaving it
                        return
                    # choose first segment and update reserved areas
                    segments = segments[0]
                    rem_area = segments.buffer(3.5)
                    if self.reserved_areas.get(car,0)!=0:
                        self.reserved_areas.update({car: rem_area})
                    elif self.reserved_areas_requested.get(car,0)!=0:
                        self.reserved_areas_requested.update({car: rem_area})

    def update_reserved_areas(self): # check this again
        items_to_delete=[]
        # find if a reserved area is in requested and reserved list, should only happen one at a time due to update fcn
        rep_key = None
        accept = True
        for key_req,val_req in self.reserved_areas_requested.items():
            for key,val in self.reserved_areas.items():
                if key_req == key:
                    rep_key = key
                    break
        # if yes check if it intersects another requested area,
        if rep_key:
            rep_val = self.reserved_areas_requested.get(rep_key)
            for key,val in self.reserved_areas.items():
                if rep_key != key:
                    if rep_val.intersects(val):
                        accept = False
            self.reserved_areas.pop(rep_key)
            if accept: # put on reserved list and delete from requested list
                self.reserved_areas.update({rep_key: rep_val})
                self.reserved_areas_requested.pop(rep_key)
            else:
                self.reserved_areas_requested.move_to_end(rep_key, False) # move to the top of the ordered dict in request list
        # check if every value in requested list intersects any of the reserved entries or entries higher up in requested
        for key_req,val_req in self.reserved_areas_requested.items():
            accept = True
            for key,val in self.reserved_areas.items():
                if val_req.intersects(val):
                    accept = False
                    break
            for key_before,val_before in self.reserved_areas_requested.items():
                if key_req == key_before:
                    break
                elif val_req.intersects(val_before):
                    accept = False
                    break
            if accept:
                self.reserved_areas.update({key_req: val_req})
                items_to_delete.append(key_req)
                key_req.reserved = True
        # remove the accepted entries from requested ordered dict
        for item in items_to_delete:
            self.reserved_areas_requested.pop(item)
        print('Areas Reserved:')
        print(self.reserved_areas)
        keys_res = []
        for key in self.reserved_areas.keys():
            print(key.id)
            keys_res.append(key.id)
        print('Areas Requested:')
        print(self.reserved_areas_requested)
        keys_req = []
        for key in self.reserved_areas_requested.keys():
            print(key.id)
            keys_req.append(key.id)
        self.Logger.info('GAME - Reserved: {0}, Requested: {1}'.format(keys_res,keys_req))

    def release_reserved_area(self,car):
        self.Logger.info('GAME - {0} releasing reserved area'.format(car.id))
        if car in self.reserved_areas:
            self.reserved_areas.pop(car)
        if car in self.reserved_areas_requested:
            self.reserved_areas_requested.pop(car)
        if car in self.trajs:
            self.trajs.pop(car)
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
                self.Logger.info('GAME - Car {} still in reserved area'.format(car.id))
                return False
        except:
            st()
        return True

    def get_vision_cone(self,car):
        buffer = abs(car.v)*3.6/10*0.4*2
        # if car.replan and car.last_segment:# and not car.retrieving:
        #     openangle = 30
        #     length = 3
        if car.last_segment and not car.retrieving:
            openangle = 45
            length = 4 + buffer # m
        elif car.unparking:
            openangle = 60
            length = 4 + buffer # m # changes here to try
        elif car.close:# or car.replan:
            openangle = 30
            length = 5.0 + buffer # m
        elif car.reserved and car.reverse:
            openangle = 30
            length = 4.0 + buffer # m
        else:
            openangle = 30
            length = 6 + buffer # m
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking and not car.last_segment:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def get_vision_cone_blocked(self,car):
        buffer = abs(car.v)/10*0.4
        openangle = 45
        length = 8 + buffer # m # was 6
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def get_vision_cone_obs(self,car):
        buffer = abs(car.v)/10*0.4
        openangle = 45
        length = 4.5 + buffer # m
        cone = Polygon([(car.x,car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,np.sin(np.deg2rad(openangle/2))*length+car.y),(car.x+np.cos(np.deg2rad(openangle/2))*length,-np.sin(np.deg2rad(openangle/2))*length+car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def get_vision_cone_pedestrian(self,car):
        if car.direction == -1:
            length = 3 # m
        else:
            length = 6 # m
        cone = Polygon([(car.x,car.y),(car.x,car.y+1),(car.x+length,car.y+1),(car.x+length,car.y-1),(car.x,car.y-1),(car.x,car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def get_forward_vision_cone(self,car):
        length = 6 # m
        cone = Polygon([(car.x,car.y),(car.x,car.y+1),(car.x+length,car.y+1),(car.x+length,car.y-1),(car.x,car.y-1),(car.x,car.y)])
        rotated_cone = affinity.rotate(cone, np.rad2deg(car.yaw), origin = (car.x,car.y))
        if car.direction == -1 or car.unparking:
            rotated_cone = affinity.rotate(rotated_cone, np.rad2deg(np.pi), origin = (car.x,car.y))
        return rotated_cone

    def check_car_path(self,car):
        conflict_cars = []
        failed_car = False
        blocked = False
        blocked_by = False
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
                                if cars.status=='Failure':# or cars.status =='Blocked':
                                    blocked = True
                                    blocked_by = cars
                                    self.Logger.info('GAME - Failure or blocked car {0} ahead of Car {1}'.format(cars.id, car.id))
        clearpath = True
        mysmallcone = self.get_forward_vision_cone(car)
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    #clearpath = False
                    for cars in self.cars:
                        if cars.name == key:
                            if cars.status=='Failure':
                                #failed_cars.append(cars)
                                failed_car = True
                                if mysmallcone.intersects(val):
                                    self.Logger.info('GAME - {0} blocked by a failed car, ID {1}'.format(car.name, car.id))
                                    clearpath = False
                            elif cars.parked:
                                self.Logger.info('GAME - Blocked by a parked car - Go on ID {0}'.format(car.id))
                                #clearpath = True
                            else:
                                conflict_cars.append(cars)
                                clearpath = False
        # check if an obstacle is in the way
        myconeobs = self.get_vision_cone_obs(car)
        for key,val in self.obstacles.items():
            if myconeobs.intersects(val):
                clearpath = False
                self.Logger.info('GAME - Blocked by an obstacle ID {0}'.format(car.id))
                #st()
        # check if a pedestrian is in the cone and I am not in the pedestrian's way
        mypedcone = self.get_vision_cone_pedestrian(car)
        for ped in self.peds:
            x_m = ped.state[0]/SCALE_FACTOR_SIM
            y_m = ped.state[1]/SCALE_FACTOR_SIM
            ped_point = Polygon([(x_m+2,y_m+1),(x_m+2,y_m-1),(x_m-1,y_m-1), (x_m-1,y_m+1),(x_m+2,y_m+1)])
            if ped_point.intersects(mypedcone) and not car.last_segment and not mybox.intersects(ped_point):
                clearpath = False
                self.Logger.info('GAME - {0} stops because a pedestrian is in the path, ID {1}'.format(car.name, car.id))
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
                            self.Logger.info('GAME - {0} stopped because of reserved area for Car {1} ahead'.format(car.id,key.id))
                    else:
                        stop_reserved = True
                        clearpath = False
                        self.Logger.info('GAME - {0} stopped because of reserved area for Car {1} ahead'.format(car.id,key.id))
                    self.Logger.info('GAME - Stopping for reserved area ID {0}'.format(car.id))
        return clearpath, conflict_cars, failed_car, conflict, blocked, stop_reserved, blocked_by

    def clear_before_unparking(self,car):
        self.update_car_boxes()
        mycone = self.get_vision_cone(car)
        for key,val in self.car_boxes.items():
            if key != car.name:
                if mycone.intersects(val):
                    self.Logger.info('GAME - {0} Waiting'.format(car.name))
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

    def update_obstacles(self, obs):
        self.obstacles.clear()
        for key,val in obs.items():
            self.obstacles.update({key: Point(val[0],val[1]).buffer(val[3])})

    def dropoff_free(self):
        self.update_car_boxes()
        for key,val in self.car_boxes.items():
            if self.dropoff_box.intersects(val):
                return False
        return True

    def is_car_at_pickup(self,car):
        self.update_car_boxes()
        mybox = self.car_boxes.get(car.name)
        if self.pickup_box.intersects(mybox):
            self.Logger.info('GAME - {0} is at pick up, ID {1}'.format(car.name,car.id))
            return True
        return False

    def is_car_in_spot(self,car):
        self.update_car_boxes()
        mycar_box = self.car_boxes.get(car.name)
        for park_box in self.park_boxes:
            if mycar_box.intersects(park_box):
                return True
        return False

    def is_car_close_2_spot(self,car):
        self.update_car_boxes()
        mycar_box = self.car_boxes.get(car.name)
        for park_box in self.park_boxes_area:
            if mycar_box.intersects(park_box):
                return True
        return False

    def is_failure_acceptable(self, obsdict):
        for key,val in obsdict.items():
            if not self.accept_box.contains(Point(val[0],val[1]).buffer(3)):
                return False
        return True

    async def run(self, Logger):
        self.Logger = Logger
        self.Logger.info('GAME - started')
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.keep_track_influx)
            nursery.start_soon(self.keep_track_influx_peds)
            nursery.start_soon(self.keep_track_outflux)
