from components.boxcomponent import BoxComponent
import trio
from variables.global_vars import *
from prepare.communication import *
import motiontracking.mpc_tracking as tracking
import math
from components.game import Game
import random
from ipdb import set_trace as st
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon
from shapely import affinity
from ipdb import set_trace as st


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class Car(BoxComponent):
    def __init__(self, arrive_time, depart_time):
        super().__init__()
        self.name = 'Car {}'.format(id(self))
        self.arrive_time = arrive_time
        self.depart_time = depart_time
        self.ref = None
        self.x = START_X
        self.y = START_Y
        self.yaw = START_YAW
        self.v = 0.0
        self.state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.v)
        self.status = 'Idle'
        self.last_segment = False
        self.direction = 1
        self.delay = 0
        self.unparking = False
        self.waiting = False
        self.replan = False
        self.close = False
        self.parking = True
        self.retrieving = False
        self.is_at_pickup = False
        self.parked = False
        self.cancel = False
        self.requested = False
        self.current_segment = None
        self.id = 0
        self.reserved = False
        self.area_requested = False
        self.picked_up = False

    async def update_planner_command(self,send_response_channel,Game, Time): # directive/response system - receiving directives
        async with self.in_channels['Planner']:
            async for directive in self.in_channels['Planner']:
                if directive == 'Wait':
                    print('{0} - Receiving Directive from Planner to wait'.format(self.name))
                    self.status = 'Waiting'
                elif directive == 'Back2spot':
                    print('{0} - Receiving Directive from Planner to drive back into the spot, ID {1}'.format(self.name, self.id))
                    directive = self.ref[:][0]
                    directive = [np.flip(directive, 0)]
                    # self.ref = directive
                    #print(directive[-1])
                    direc = [[self.x/SCALE_FACTOR_PLAN, self.y/SCALE_FACTOR_PLAN, -1*np.rad2deg(self.yaw)]]
                    direc.append([directive[-1][-1][0], directive[-1][-1][1], directive[-1][-1][2]] )
                    directive = [np.array(direc)]
                    self.ref = directive
                    print('Tracking this path:')
                    print(self.ref)
                    # st()
                    self.replan = True
                    self.last_segment = True
                    self.direction = 1
                    await self.track_reference(Game,send_response_channel, Time)
                    await trio.sleep(0)
                elif directive == 'Reverse':
                    print('{0} - Receiving Directive from Planner to reverse'.format(self.name))
                    directive = self.current_segment
                    directive = [np.flip(directive, 0)]
                    direc = [[self.x/SCALE_FACTOR_PLAN, self.y/SCALE_FACTOR_PLAN, -1*np.rad2deg(self.yaw)]]
                    direc.append([directive[-1][-1][0], directive[-1][-1][1], directive[-1][-1][2]] )
                    directive = [np.array(direc)]
                    self.ref = directive
                    print('Tracking this path:')
                    print(self.ref)
                elif directive == 'OriginalPath':
                    print('Tracking the original path and wait for failure to be removed ID {0}'.format(self.id))
                    print(self.ref)
                    await self.track_reference(Game,send_response_channel, Time)
                elif not directive:
                    trio.sleep(0)
                elif not self.picked_up:
                    print('{0} - Receiving Directive from Planner'.format(self.name))
                    self.ref = np.array(directive)
                    print(self.ref)
                    #if self.replan:
                    #print('Tracking this path:')
                    #print(directive)
                    #if directive=='Unpark':
                    #self.unparking = True
                    await self.track_reference(Game,send_response_channel, Time)
                    await trio.sleep(0)
                    #await self.send_response(send_response_channel)

    async def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        if oa is None or od is None:
            oa = [0.0] * tracking.T
            od = [0.0] * tracking.T
        for i in range(tracking.MAX_ITER):
            xbar = tracking.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, _, _, _, _ = tracking.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= tracking.DU_TH:
                break
        return oa, od

    async def stop_car(self): # bringing car to a full stop asap
        if not self.status == 'Stop':
        # bring car to a full stop
            odelta, oa = None, None
            buffer = self.v / 10 * 0.4
            cx = [self.x, self.x + 0.5 * self.direction*buffer*np.cos(self.yaw), self.x + self.direction*buffer*np.cos(self.yaw)]
            cy = [self.y, self.y + 0.5 * self.direction*buffer*np.sin(self.yaw), self.y + self.direction*buffer*np.sin(self.yaw)]
            cyaw = [self.yaw, self.yaw, self.yaw]
            ck = 0
            dl = 1.0
            goal = [cx[-1], cy[-1]]
            target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
            sp = [self.v, self.v/2, 0]
            xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)
            x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
            oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
            if odelta is not None:
                di, ai = odelta[0], oa[0]
            self.state = tracking.update_state(self.state, ai, di)
            self.x = self.state.x
            self.y = self.state.y
            self.yaw = self.state.yaw
            self.v = self.state.v

    async def track_async(self, cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed,Game,send_response_channel,Time): # modified from MPC
        goal = [cx[-1], cy[-1]]
        self.state = initial_state
        # initial yaw compensation
        if self.state.yaw - cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0
        time = 0.0
        if self.reserved:
            if self.clear_of_reserved_area(Game):
                self.release_reverse_area(Game,send_response_channel)
                self.reserved = False
        target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
        odelta, oa = None, None
        cyaw = tracking.smooth_yaw(cyaw)
        self.waiting = False
        blocked = False
        while tracking.MAX_TIME >= time:
            if self.status == 'Removed':
                print('{0} Removed'.format(self.name))
                self.v = 0
                return
            elif self.status == 'Replan':
                    print('{0} Stopping the Tracking, ID {1}'.format(self.name, self.id))
                    await self.stop_car()
                    #self.v = 0 # include braking here
                    return
            while not self.path_clear(Game) or blocked:
                await self.stop_car()
                self.status = 'Stop'
                if self.status == 'Removed':
                    print('{0} Removed'.format(self.name))
                    self.v = 0
                    return
                print('{0} stops because path is blocked, ID {1}'.format(self.name, self.id))
                _, conflict_cars, failed, conflict, blocked, stop_reverse = self.check_path(Game)
                if stop_reverse:
                    print('BBBBBBBBBB ---- {0} stopped because of reserved area ahead'.format(self.id))
                # insert here for replanning!!
                # self.update_delay(Time)
                # if self.delay > DELAY_THRESH and not self.area_requested:
                #     await self.request_reserved_area(send_response_channel)
                #print(conflict_cars)
                #print(failed)
                #print(self.waiting)
                #if not self.status == 'Waiting':
                if conflict and not self.waiting:
                    self.status = 'Conflict'
                    #send response to sup
                    print('We have a conflict')
                    await self.send_conflict(conflict_cars, send_response_channel)
                    self.waiting = True
                        # return
                if failed or blocked and not self.waiting and not self.replan:
                    #send response to sup
                    self.status = 'Blocked'
                    print('Blocked by a failure')
                    await self.send_response(send_response_channel)
                    self.waiting = True
                    # return
                if self.status == 'Replan':
                    print('{0} Stopping the Tracking, ID {1}'.format(self.name, self.id))
                    self.v = 0 # include braking here
                    return
                await trio.sleep(3)
            self.status = 'Driving'
            self.waiting = False
            xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)
            x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
            oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
            if odelta is not None:
                di, ai = odelta[0], oa[0]
            self.state = tracking.update_state(self.state, ai, di)
            time = time + tracking.DT
            self.x = self.state.x
            self.y = self.state.y
            self.yaw = self.state.yaw
            self.v = self.state.v
            await trio.sleep(0)
            if tracking.check_goal(self.state, goal, target_ind, len(cx),goalspeed,self.last_segment): # modified goal speed
                break

    def update_delay(self,Time):
        if self.requested:
            now = trio.current_time()
            now_del = now-Time.START_TIME
            print('The time is {0}'.format(now_del))
            if self.requested and self.depart_time <= now_del:
                self.delay = now_del-self.depart_time
            print('Car {0} wanted to depart at {1} and has delay {2}'.format(self.id,self.depart_time,self.delay))
        #print("deptime"+str(self.depart_time))
        #print("car.delay"+str(self.delay))
        #print("now"+str(now))
        #st()

    def release_reverse_area(self,gme,send_response_channel):
        print('Releasing the reserved area for Car {0}'.format(self.id))  
        gme.release_reverse_area(self)  
        self.area_requested = False

    async def request_reserved_area(self,send_response_channel):
        self.area_requested = True
        print('AAAAAAAAAAAAAAA Requesting reserved area for Car ID {0} due to delay {1}'.format(self.id, self.delay))   
        response = 'RequestReservedArea'
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
 
    async def track_reference(self,Game,send_response_channel, Time):
        now = trio.current_time()
        # if self.depart_time + Time.START_TIME <= now:
        #     self.delay = (now-Time.START_TIME)-self.depart_time
        print('{0} - Tracking reference...'.format(self.name))
        #print(self.ref)
        # print("deptime"+str(self.depart_time))
        # print("car.delay"+str(self.delay))
        # print("now"+str(now))
        self.update_delay(Time)
        try:
            self.current_segment = self.ref[:][0]
        except:
            st()
        print("{0} delay: {1}".format(self.name,self.delay))
        if self.delay > DELAY_THRESH and not self.area_requested:
            await self.request_reserved_area(send_response_channel)
        #print("car.delay: "+str(self.delay))
        self.close = False
        ck = 0 
        dl = 1.0  # course tick
        # check if car shoud be removed
        if self.status == 'Removed':
            print('{0} Removed'.format(self.name))
            self.v = 0
            return
        if not self.status == 'Replan':
            try:
                self.check_if_car_is_in_spot(Game)
            except: 
                st()
            if self.check_if_car_is_in_spot(Game):
                print('Car is in a parking spot')
                self.parked = True
                while not self.check_clear_before_unparking(Game):
                    await trio.sleep(0.1)
        self.status = 'Driving'
        self.parked = False
        # including a failure in 20% of cars
        failidx = len(self.ref)
        chance = random.randint(1,100) # changed to 0!!!
        if self.id == 0:
            chance = 1
        if not self.replan:
            if len(self.ref)-1>4 and chance <=30:
                failidx = np.random.randint(low=4, high=6, size=1)
                if self.parking:
                    print('{0} will fail at acceptable spot: {1}'.format(self.name,failidx))
                else:
                    print('{0} will fail in narrow path: {1}'.format(self.name,failidx))
            elif len(self.ref)-1>10 and chance <=30:
                failidx = np.random.randint(low=len(self.ref)-5, high=len(self.ref)-1, size=1)
                if self.parking:
                    print('{0} will fail in narrow path: {1}'.format(self.name,failidx))
                else:
                    print('{0} will fail at acceptable spot: {1}'.format(self.name,failidx))
        # start tracking segments
        elif self.status == 'Replan':
            print('{0} Stopping the Tracking, ID {1}'.format(self.name, self.id))
            await self.stop_car()
            #self.v = 0 # include braking here
            return
        for i in range(0,len(self.ref)-1):
            #print('{0} self.unparking'.format(self.name))
            #print(self.unparking)
            if (i==failidx):
                print('{0} Failing'.format(self.name))
                await self.failure(send_response_channel)
                return  
            if i >= 1:
                self.unparking = False
            self.close = False
            if self.check_car_close_2_spot(Game):
                self.close = True
            self.status = 'Driving'
            if self.ref == None:
                await self.stop_car()
                return
            path = self.ref[:][i]
            self.current_segment = path
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            state = np.array([self.x, self.y,self.yaw])
            #  check  direction of the segment
            self.direction = tracking.check_direction(path) 
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,TARGET_SPEED,self.direction)
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,TARGET_SPEED,Game,send_response_channel,Time)
            await trio.sleep(0)
            if self.status == 'Replan' or self.status=='Removed':
                #self.ref = None
                return
        if not self.status == 'Failure':
            self.last_segment = True
            state = np.array([self.x, self.y,self.yaw])
            path = self.ref[:][-1]
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            self.direction = tracking.check_direction(path)
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,0.0,self.direction)
            #print('sp')
            #print(sp)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,0.0,Game,send_response_channel,Time)
            if self.status == 'Replan' or self.status=='Removed':
                #self.ref = None
                return
            self.status = 'Completed'
            self.is_at_pickup = self.check_at_pickup(Game)
            if self.is_at_pickup:
                self.retrieving = False
            self.last_segment = False
            if self.check_if_car_is_in_spot(Game):
                self.parked = True
                print('Car is in a parking spot')
            self.parking = False
            self.ref = None
            await self.send_response(send_response_channel)

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = self.status
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def send_conflict(self,cars,send_response_channel):
        response = (self.status,cars)
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    def clear_of_reserved_area(self,gme):
        free = gme.is_car_free_of_reserved_area(self)
        return free

    def path_clear(self, gme):
        clear, _, _ ,_,_,_= self.check_path(gme)
        return clear
    
    def check_clear_before_unparking(self,gme):
        self.unparking = True
        clear = gme.clear_before_unparking(self)
        return clear

    def check_car_close_2_spot(self,gme):
        close = gme.is_car_close_2_spot(self)
        return close

    def check_at_pickup(self,gme):
        at_pickup = gme.is_car_at_pickup(self)
        return at_pickup

    def check_path(self, gme):
        #print('Checking the path')
        clear, conflict_cars, failed, conflict, blocked, stop_reserved = gme.check_car_path(self)
        return clear, conflict_cars, failed, conflict, blocked, stop_reserved

    def check_if_car_is_in_spot(self,gme):
        in_spot = gme.is_car_in_spot(self)
        return in_spot

    # async def stop(self,send_response_channel):
    #     self.status = 'Stop'
    #     await self.send_response(send_response_channel)

    async def failure(self,send_response_channel):
        self.status = 'Failure'
        await self.send_response(send_response_channel)
        await trio.sleep(1000) # freeze car

    # async def cancel_car(self,nursery):
    #     while True:
    #         if self.cancel:
    #             print('Cancelling {0}'.format(self.name))
    #             nursery.cancel_scope.cancel()


    async def run(self,send_response_channel,Game, Time):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command,send_response_channel,Game, Time)
            if self.cancel:
                print('Cancelling {0}'.format(self.name))
                nursery.cancel_scope.cancel()
            await trio.sleep(0)