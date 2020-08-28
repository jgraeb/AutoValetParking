# Automated Valet Parking - Car Module
# Josefine Graebener
# California Institute of Technology
# March, 2020

from prepare.boxcomponent import BoxComponent
import trio
import rospy
import numpy as np
from variables.global_vars import START_X, START_Y, START_YAW, SCALE_FACTOR_PLAN, DELAY_THRESH, TARGET_SPEED, SCALE_FACTOR_PLAN, accSteerCtrl, velCtrl, turtlebotCtrl, accRotVelCtrl
#from prepare.communication import  
import motiontracking.mpc_tracking as tracking
import math
from components.game import Game
from gazebo.src.RobotCtrl import RobotCtrl
from gazebo.src.Publisher import Publisher
from prepare.communication import get_current_time
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
    def __init__(self, arrive_time, depart_time, car_nbr):
        super().__init__()
        self.robot_ctrl = RobotCtrl()
        self.publisher = Publisher(START_X, START_Y, START_YAW)
        self.car_nbr = car_nbr
        self.name = 'Car {}'.format(id(self))
        self.arrive_time = arrive_time
        self.depart_time = depart_time
        self.ref = []
        self.x = START_X
        self.y = START_Y
        self.yaw = START_YAW
        self.v = 0.0
        self.state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.v)
        self.status = 'Idle' # ('Driving','Replan','Stop','Conflict','Parked','Failure', 'Removed', 'Waiting')
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
        self.in_spot = False
        self.cancel = False
        self.requested = False
        self.current_segment = None
        self.id = 0
        self.reserved = False
        self.area_requested = False
        self.picked_up = False
        self.reverse = False
        self.idx = 0
        self.hold = False
        self.new_spot = False
        self.Logger = None

    async def update_planner_command(self,send_response_channel,Game, Time): # directive/response system - receiving directives
        async with self.in_channels['Planner']:
            async for directive in self.in_channels['Planner']:
                #st()
                if directive == 'Wait':
                    self.Logger.info('{0} - Receiving Directive from Planner to wait'.format(self.name))
                    self.status = 'Waiting'
                elif directive == 'Back2spot':
                    self.Logger.info('{0} - Receiving Directive from Planner to drive back into the spot, ID {1}'.format(self.name, self.id))
                    # directive = self.ref[:][0]
                    # directive = [np.flip(directive, 0)]
                    # direc = [[self.x/SCALE_FACTOR_PLAN, self.y/SCALE_FACTOR_PLAN, -1*np.rad2deg(self.yaw)]]
                    # direc.append([directive[-1][-1][0], directive[-1][-1][1], directive[-1][-1][2]] )
                    # directive = [np.array(direc)]
                    # self.ref = directive
                    # print('Tracking this path:')
                    # print(self.ref)
                    # self.last_segment = True
                    # self.direction = 1
                    # await self.track_reference(Game,send_response_channel, Time)
                    # await trio.sleep(0)
                elif directive == 'Reverse': # not used
                    self.Logger.info('{0} - Receiving Directive from Planner to reverse'.format(self.name))
                    st()
                    x = self.x*SCALE_FACTOR_PLAN #[self.x, self.x + 0.5 * self.direction*buffer*np.cos(self.yaw), self.x + self.direction*buffer*np.cos(self.yaw)]
                    y = self.y*SCALE_FACTOR_PLAN #[self.y, self.y + 0.5 * self.direction*buffer*np.sin(self.yaw), self.y + self.direction*buffer*np.sin(self.yaw)]
                    yaw = np.rad2deg(self.yaw)
                    direc = [[x, y, yaw]]
                    direc.append([x/SCALE_FACTOR_PLAN-5*np.cos(yaw), y/SCALE_FACTOR_PLAN-5*np.sin(yaw), yaw])
                    direc.append([x/SCALE_FACTOR_PLAN-10*np.cos(yaw), y/SCALE_FACTOR_PLAN-10*np.sin(yaw), yaw])
                    directive = np.array(direc)

                    #directive = np.array([[ x, y, yaw, 0],[ x + 0.5 * (-1)*np.cos(yaw), y + 0.5 * (-1)*np.sin(yaw), yaw,  0],[ x + (-1)*np.cos(yaw), y + (-1)*np.sin(yaw), yaw,  0]])
                    self.ref = directive
                    # print('Tracking this path:')
                    # print(self.ref)
                elif directive == 'OriginalPath':
                    self.Logger.info('{0} - Tracking the original path and wait for failure to be removed ID {0}'.format(self.name,self.id))
                    await self.track_reference(Game,send_response_channel, Time)
                elif len(directive)==0:
                    trio.sleep(0)
                elif not self.picked_up:
                    self.Logger.info('{0} - Receiving Directive from Planner'.format(self.name))
                    self.ref = np.array(directive)
                    #print(self.ref)
                    #if self.replan:
                    #print('Tracking this path:')
                    #print(directive)
                    #if directive=='Unpark':
                    #self.unparking = True
                    await self.track_reference(Game,send_response_channel, Time)
                    await trio.sleep(0)
                    #await self.send_response(send_response_channel)

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od, h ,breaking):
        if oa is None or od is None:
            oa = [0.0] * tracking.T
            od = [0.0] * tracking.T
        for _ in range(tracking.MAX_ITER):
            xbar = tracking.predict_motion(x0, oa, od, xref, h)
            poa, pod = oa[:], od[:]
            oa, od, _, _, _, _ = tracking.linear_mpc_control(xref, xbar, x0, dref, h, breaking)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= tracking.DU_TH:
                break
        return oa, od

    async def stop_car(self): # bringing car to a full stop asap
        #original version
        if not self.status == 'Stop' and not self.status=='Blocked' and not self.status=='Conflict':
        # bring car to a full stop
            if self.v == 0: # if car is already stopped
                return
            if self.status != 'Replan':
                self.status = 'Stop'
            print('STOPPING CAR {0}, velocity {1}'.format(self.id, self.v))
            #st()
            # make up stopping traj
            buffer = self.v * 3.6 / 10 * 0.4 * 2 # formula for breaking distance plus 100 % safety margin
            cx = [self.x, self.x + 0.5 * self.direction*buffer*np.cos(self.yaw), self.x + self.direction*buffer*np.cos(self.yaw)]
            cy = [self.y, self.y + 0.5 * self.direction*buffer*np.sin(self.yaw), self.y + self.direction*buffer*np.sin(self.yaw)]
            cyaw = [self.yaw, self.yaw, self.yaw]
            # designate goal
            goal = [cx[-1], cy[-1]]
            # set speed profile
            sp = [self.v, 0, 0]
            # define initial state - car current state
            state = np.array([self.x, self.y,self.yaw])
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            self.state = initial_state
            # initial yaw compensation
            if self.state.yaw - cyaw[0] >= math.pi:
                self.state.yaw -= math.pi * 2.0
            elif self.state.yaw - cyaw[0] <= -math.pi:
                self.state.yaw += math.pi * 2.0
            time = 0.0
            x = [self.state.x]
            y = [self.state.y]
            yaw = [self.state.yaw]
            v = [self.state.v]
            #vkmh = [state.v*3.6] # mod storing speed in km/h
            t = [0.0]
            d = [0.0]
            a = [0.0]
            target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
            odelta, oa = None, None

            cyaw = tracking.smooth_yaw(cyaw)
            ck = 0
            dl = 1.0
            while tracking.MAX_TIME >= time:
                xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)
                # current state
                x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
                oa, odelta = self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta, True)
                if odelta is not None:
                    di, ai = odelta[0], oa[0]
                self.state = tracking.update_model_state(self.state, ai, di)
                self.x = self.state.x
                self.y = self.state.y
                self.yaw = self.state.yaw
                self.v = self.state.v
                time = time + tracking.DT
                #self.status == 'Stop'
                if tracking.check_goal(self.state, goal, target_ind, len(cx),0,self.last_segment): # modified goal speed
                    self.v = 0
                    break

        else:
            self.v = 0

        # version adapted for turtlebots AccSteerCtrl, not tested
       # if not self.status == 'Stop' and not self.status=='Blocked' and not self.status=='Conflict':
       # # bring car to a full stop
       #     if self.v == 0: # if car is already stopped
       #         return
       #     if self.status != 'Replan':
       #         self.status = 'Stop'
       #     print('STOPPING CAR {0}, velocity {1}'.format(self.id, self.v))
       #     # make up stopping traj
       #     buffer = self.v * 3.6 / 10 * 0.4 * 2 # formula for breaking distance plus 100 % safety margin
       #     cx = [self.x, self.x + 0.5 * self.direction*buffer*np.cos(self.yaw), self.x + self.direction*buffer*np.cos(self.yaw)]
       #     cy = [self.y, self.y + 0.5 * self.direction*buffer*np.sin(self.yaw), self.y + self.direction*buffer*np.sin(self.yaw)]
       #     cyaw = [self.yaw, self.yaw, self.yaw]
       #     # designate goal
       #     goal = [cx[-1], cy[-1]]
       #     # set speed profile
       #     sp = [self.v, 0, 0]
       #     # define initial state - car current state
       #     state = np.array([self.x, self.y,self.yaw])
       #     initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
       #     self.state = initial_state
       #     # initial yaw compensation
       #     if self.state.yaw - cyaw[0] >= math.pi:
       #         self.state.yaw -= math.pi * 2.0
       #     elif self.state.yaw - cyaw[0] <= -math.pi:
       #         self.state.yaw += math.pi * 2.0
       #     time = 0.0
       #     x = [self.state.x]
       #     y = [self.state.y]
       #     yaw = [self.state.yaw]
       #     v = [self.state.v]
       #     #vkmh = [state.v*3.6] # mod storing speed in km/h
       #     t = [0.0]
       #     d = [0.0]
       #     a = [0.0]
       #     target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
       #     odelta, oa = None, None
#
       #     cyaw = tracking.smooth_yaw(cyaw)
       #     ck = 0
       #     dl = 1.0
       #     start_time = trio.current_time()
       #     iterations = 0
       #     while tracking.MAX_TIME >= time:
       #         iterations += 1 
       #         self.x, self.y, self.yaw = self.robot_ctrl.get_pos(self.car_nbr)
       #         #self.yaw = -self.yaw
       #         self.v, _ = self.robot_ctrl.get_vel(self.car_nbr)
       #         self.state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.v)
       #         xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)
       #         if tracking.check_goal(self.state, goal, target_ind, len(cx),0,self.last_segment): # modified goal speed
       #             self.robot_ctrl.set_vel(self.car_nbr, 0, 0)
       #             self.v = 0
       #             break
       #         # current state
       #         x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state 
       #         oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta, tracking.DT, True)
       #         if odelta is not None:
       #             di, ai = odelta[0], oa[0]
       #             self.robot_ctrl.set_acc_n_steer(self.car_nbr, ai, di)
       #         time = time + tracking.DT
       #         #self.status == 'Stop'
       #         if(trio.current_time() < start_time + iterations*tracking.DT):
       #             await trio.sleep_until(start_time + iterations*tracking.DT)
       # else:
       #     self.robot_ctrl.set_vel(self.car_nbr, 0, 0)
       #     self.v = 0


    async def back_2_spot(self,Time,send_response_channel,Game): #not adapted for turtlebots
        #st()
        self.status = 'Replan'
        self.replan = True
        directive = self.ref[:][0]
        directive = [np.flip(directive, 0)]
        direc = [[self.x/SCALE_FACTOR_PLAN, self.y/SCALE_FACTOR_PLAN, -1*np.rad2deg(self.yaw)]]
        direc.append([directive[-1][-1][0], directive[-1][-1][1], directive[-1][-1][2]] )
        directive = [np.array(direc)]
        #self.ref = directive
        # print('Tracking this path:')
        # print(directive)
        self.last_segment = True
        self.direction = 1
        self.update_delay(Time)
        self.current_segment = directive
        self.Logger.info('{0} - delay: {1}'.format(self.name,self.delay))
        if self.delay > DELAY_THRESH and not self.area_requested:
            await self.request_reserved_area(send_response_channel)
        ck = 0 
        dl = 1.0  # course tick             
        # check if car shoud be removed
        if self.status == 'Removed':
            print('{0} Removed'.format(self.name))
            self.v = 0
            return
        self.status = 'Driving'
        self.in_spot = False
        self.parked = False
        # including a failure in 20% of cars
        if self.status == 'Replan':
            self.Logger.info('{0} - Stopping the Tracking, ID {1}'.format(self.name, self.id))
            await self.stop_car()
            return
        if not self.status == 'Failure' and len(directive)!=0:
            self.last_segment = True
            self.idx = 0
            path = directive
            cx = [entry[0]*SCALE_FACTOR_PLAN for entry in path[0]] 
            cy = [entry[1]*SCALE_FACTOR_PLAN for entry in path[0]]
            cyaw = [np.deg2rad(entry[2])*-1 for entry in path[0]]
            self.direction = tracking.check_direction(path[0])
            state = np.array([self.x, self.y,self.yaw])
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            end_speed = 0.0
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,end_speed,self.direction)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,end_speed,Game,send_response_channel,Time)
            if self.status == 'Replan' or self.status=='Removed' or self.status=='Blocked':
                return
            self.status = 'Completed'
            self.is_at_pickup = self.check_at_pickup(Game)
            if self.is_at_pickup:
                self.retrieving = False
            self.last_segment = False
            if self.check_if_car_is_in_spot(Game):
                self.in_spot = True
            self.parking = False
        self.Logger.info('{0} - back in spot'.format(self.name))
        await trio.sleep(0)
        await self.track_reference(Game, send_response_channel,Time)

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
                self.release_reserved_area(Game,send_response_channel)
                self.reserved = False
        target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
        odelta, oa = None, None
        cyaw = tracking.smooth_yaw(cyaw)
        #self.waiting = False
        blocked = False
        start_time = trio.current_time()
        iterations = 0
        ai = 0
        di = 0
        time_last = 0
        while tracking.MAX_TIME >= time:
            iterations += 1 
            if self.status == 'Removed':
                self.Logger.info('{0} - Removed'.format(self.name))
                self.v = 0
                return
            elif self.status == 'Replan':
                    self.Logger.info('{0} Stopping the Tracking, ID {1}, A'.format(self.name, self.id))
                    await self.stop_car()
                    return
            while self.hold:
                self.status = 'Stop'
                self.Logger.info('{0} holding for other lane to clear'.format(self.name))
                await trio.sleep(3)
                if Game.is_reserved_area_clear(self):
                    self.hold = False
                    break
            while not self.path_clear(Game):# or blocked:
                self.hold = False
                print("path not clear")
                await self.stop_car()
                if self.status == 'Removed':
                    self.Logger.info('{0} - Removed'.format(self.name))
                    self.v = 0
                    return
                self.Logger.info('{0} - stops because path is blocked, ID {1}'.format(self.name, self.id))
                _, conflict_cars, failed_car, conflict, blocked, stop_reserved, blocked_by = self.check_path(Game)
                if stop_reserved:
                    self.status = 'Stop'
                    self.Logger.info('{0} STOP ---- stopped because of reserved area ahead'.format(self.name))
                    if not self.area_requested:
                        # request reserved area
                        await self.request_area(send_response_channel)
                if self.reverse:
                    self.waiting = True
                    self.Logger.info('{0} - sending max reverse'.format(self.name))
                    await self.send_max_reverse(send_response_channel)
                    self.status == 'Replan'
                    return
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
                    #send response to supervisor
                    self.Logger.info('{0} - We have a conflict'.format(self.name))
                    if self.unparking:
                        try:
                            self.Logger.info('{0} - Driving back into the spot'.format(self.name))
                            await self.back_2_spot(Time,send_response_channel,Game)
                            self.replan = True
                            return
                        except:
                            st()
                        #await self.track_reference(Game,send_response_channel, Time)
                        return
                    await self.send_conflict(conflict_cars, send_response_channel)
                    self.waiting = True
                    if self.reverse:
                        self.Logger.info('{0} - sending max reverse'.format(self.name))
                        await self.send_max_reverse(send_response_channel)
                    # return
                # elif failed_car or blocked and self.replan and not self.waiting:
                #     self.status = 'Blocked'
                #     await self.send_blocked_again(blocked_by, send_response_channel)
                #     self.waiting = True
                #     return
                elif (failed_car or blocked) and (not self.waiting or self.replan):# and not self.replan:
                    #send response to sup
                    self.status = 'Blocked'
                    self.Logger.info('{0} - Blocked by a failure'.format(self.name))
                    self.waiting = True
                    if not self.replan:
                        await self.send_blocked(blocked_by, send_response_channel)
                    elif self.reverse:
                        self.Logger.info('{0} - sending max reverse'.format(self.name))
                        await self.send_max_reverse(send_response_channel)
                    elif self.replan:
                        #st()
                        await self.send_blocked_again(blocked_by, send_response_channel)
                        self.Logger.info('{0} - Stopping the Tracking, ID {1}, B'.format(self.name, self.id))
                        return
                if self.status == 'Replan':
                    self.Logger.info('{0} - Stopping the Tracking, ID {1}, C'.format(self.name, self.id))
                    #self.v = 0 # include full stop braking here
                    await self.stop_car()
                    return
                await trio.sleep(3)
            #print('Car {} back to driving'.format(self.id))
            self.status = 'Driving'
            self.waiting = False

            #used for calculating the one step prediction error
            #model_state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.v)                    
            #predicted_state = tracking.update_model_state(model_state, ai, di, tracking.DT)
            
            self.x, self.y, self.yaw = self.robot_ctrl.get_pos(self.car_nbr)
            self.v, _ = self.robot_ctrl.get_vel(self.car_nbr)
            self.state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.v)

            #For using the predicted states instead of the measured states.   
            #self.state = tracking.update_model_state(self.state, ai, di, tracking.DT)
            #self.x = self.state.x
            #self.y = self.state.y
            #self.yaw = self.state.yaw
            #self.v = self.state.v

            #self.publisher.publish_one_step_pred_error(self.x, self.y, self.v, self.yaw, predicted_state.x,
            #                                            predicted_state.y, predicted_state.v, predicted_state.yaw)

            xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)

            if tracking.check_goal(self.state, goal, target_ind, len(cx),goalspeed,self.last_segment): # modified goal speed
                if self.last_segment:
                    self.robot_ctrl.set_vel(self.car_nbr, 0, 0)
                break
            
            x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
            
            #For using the last sample time in the optimization
            #time_now = trio.current_time()
            #if time_last == 0:
            #    h = tracking.DT
            #else: 
            #    h = time_now - time_last
           
            #For using DT as sample time
            h = tracking.DT

            oa, odelta = self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta, h, False)
            
            #time_last = time_now
            
            #computing_time = trio.current_time()-time_now
            #self.publisher.publish_float(computing_time)
            
            if odelta is not None:
                di, ai = odelta[0], oa[0]
                if accSteerCtrl:
                    self.robot_ctrl.set_acc_n_steer(self.car_nbr, ai, di)
                elif velCtrl:
                    self.robot_ctrl.set_vel_n_steer(self.car_nbr, ai, di)
                elif turtlebotCtrl:
                    self.robot_ctrl.set_vel(self.car_nbr, ai, di)
                elif accRotVelCtrl:
                    self.robot_ctrl.set_acc_n_rot_vel(self.car_nbr, ai, di)

            time = time + tracking.DT

            if(trio.current_time() < start_time + iterations*tracking.DT):
                await trio.sleep_until(start_time + iterations*tracking.DT)

            

    def update_delay(self,Time):
        if self.requested:
            now = trio.current_time()
            now_del = now-Time.START_TIME
            #print('The time is {0}'.format(now_del))
            if self.requested and self.depart_time <= now_del:
                self.delay = now_del-self.depart_time
            self.Logger.info('{0} - wanted to depart at {1} and has delay {2}'.format(self.name,self.depart_time,self.delay))

    def release_reserved_area(self,Game,send_response_channel):
        if self.reserved:
            self.hold = False
            self.Logger.info('{0} - Releasing the reserved area for Car {0}'.format(self.name, self.id))  
            Game.release_reserved_area(self)  
            # self.area_requested = False
            # self.replan = False
        elif self in Game.reserved_areas_requested:
            Game.reserved_areas_requested.pop(self)
            self.Logger.info('{0} - Releasing the requested area for Car {0}'.format(self.name,self.id))  
        self.area_requested = False
        self.replan = False


    async def request_reserved_area(self,send_response_channel):
        self.area_requested = True
        self.Logger.info('{0} - Requesting reserved area for Car ID {0} due to delay {1}'.format(self.name,self.id, self.delay))   
        response = 'RequestReservedArea'
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))

    async def request_area(self,send_response_channel):
        self.area_requested = True
        self.Logger.info('{0} - Requesting reserved area for Car ID {1} due to delay {2}'.format(self.name, self.id, self.delay))   
        response = 'RequestArea'
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
 
    async def track_reference(self,Game,send_response_channel, Time):
        self.Logger.info('{0} - Tracking path'.format(self.name))
        #print(self.ref)
        self.update_delay(Time)
        try:
            self.current_segment = self.ref[:][0]
        except:
            st()
        self.Logger.info('{0} delay: {1}'.format(self.name,self.delay))
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
                self.Logger.info('{0} is in a parking spot'.format(self.name))
                #self.parked = True
                while not self.check_clear_before_unparking(Game):
                    await trio.sleep(0.1)
        self.status = 'Driving'
        self.in_spot = False
        self.parked = False
        # including a failure in 20% of cars
        failidx = len(self.ref)
        chance = 100 #random.randint(1,100) # changed to 0!!!   Runs without failures now.
        #if self.id ==1:
        #   chance = 1
        if not self.replan:
            if len(self.ref)-1>10 and chance <=30:
                failidx = np.random.randint(low=len(self.ref)-5, high=len(self.ref)-1, size=1)
                if self.parking:
                    print('{0} will fail in narrow path: {1}'.format(self.name,failidx))
                else:
                    print('{0} will fail at acceptable spot: {1}'.format(self.name,failidx))
            elif len(self.ref)-1>4 and chance <=30:
                failidx = np.random.randint(low=4, high=6, size=1)
                if self.parking:
                    print('{0} will fail at acceptable spot: {1}'.format(self.name,failidx))
                else:
                    print('{0} will fail in narrow path: {1}'.format(self.name,failidx))
        # start tracking segments
        elif self.status == 'Replan':
            print('{0} Stopping the Tracking, ID {1}'.format(self.name, self.id))
            await self.stop_car()
            return
        for i in range(0,len(self.ref)):#-1
            self.idx = i
            if (i==failidx):
                print('{0} Failing'.format(self.name))
                await self.failure(Game,send_response_channel)
                return  
            if i >= 1:
                self.unparking = False
            self.close = False
            if self.status == 'Replan' or self.status=='Removed' or self.status =='Blocked':
                return
            if self.check_car_close_2_spot(Game):
                self.close = True
            self.status = 'Driving'
            if len(self.ref)== 0: 
                await self.stop_car()
                return
            path = self.ref[:][i]
            n = len(path)
            if i == len(self.ref)-1:
                end_speed = 0
            else:
                end_speed = TARGET_SPEED
            for k in range(0,n-1): # n-1
                self.current_segment = path
                cx = path[:,0]*SCALE_FACTOR_PLAN
                cy = path[:,1]*SCALE_FACTOR_PLAN
                cyaw = np.deg2rad(path[:,2])*-1
                state = np.array([self.x, self.y,self.yaw])
                self.direction = tracking.check_direction(path) 
                sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,end_speed,self.direction)
                initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
                await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,end_speed,Game,send_response_channel,Time)
                await trio.sleep(0)
            if self.status == 'Replan' or self.status=='Removed' or self.status =='Blocked':
                return
       # if not self.status == 'Failure' and len(self.ref)!=0:
       #     self.last_segment = True
       #     self.idx = len(self.ref)-1
       #     #state = np.array([self.x, self.y,self.yaw])
       #     path = self.ref[:][-1]
       #     n = len(path)
       #     for i in range(0,n-1):
       #         #print('i'+str(i))
       #         path_seg = path[i:i+2]
       #         cx = path_seg[:,0]*SCALE_FACTOR_PLAN
       #         cy = path_seg[:,1]*SCALE_FACTOR_PLAN
       #         cyaw = np.deg2rad(path[:,2])*-1
       #         self.direction = tracking.check_direction(path)
       #         state = np.array([self.x, self.y,self.yaw])
       #         initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
       #         end_speed = TARGET_SPEED/2
       #         if i == len(path)-2:
       #             end_speed = 0.0
       #         sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,end_speed,self.direction)
       #         await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,end_speed,Game,send_response_channel,Time)
       #         if self.status == 'Replan' or self.status=='Removed' or self.status=='Blocked':
       #             return
        self.status = 'Completed'
        self.is_at_pickup = self.check_at_pickup(Game)
        if self.is_at_pickup:
            self.retrieving = False
        self.last_segment = False
        if self.check_if_car_is_in_spot(Game):
            #self.parked = True
            self.in_spot = True
            self.Logger.info('{0} is in a parking spot'.format(self.name))
        self.parking = False
        self.ref = []
        #if self.reverse:
        #    self.status = '' # change to expecting path
        await self.send_response(send_response_channel)

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = self.status
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def send_conflict(self,cars,send_response_channel):
        response = (self.status,cars)
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def send_blocked(self,cars,send_response_channel):
        response = (self.status,cars)
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        try:
            await send_response_channel.send((self,response))
        except: 
            st()
        await trio.sleep(1)

    async def send_blocked_again(self,cars,send_response_channel):
        #st()
        response = ('Reverse',cars)
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def send_max_reverse(self,send_response_channel):
        response = 'Completed'
        self.Logger.info('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    def clear_of_reserved_area(self,gme):
        free = gme.is_car_free_of_reserved_area(self)
        if not free:
            gme.update_reserved_area_for_car(self)
        return free

    def path_clear(self, gme):
        clear,_,_,_,_,_,_= self.check_path(gme)
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
        clear, conflict_cars, failed, conflict, blocked, stop_reserved, blocked_by = gme.check_car_path(self)
        return clear, conflict_cars, failed, conflict, blocked, stop_reserved, blocked_by

    def check_if_car_is_in_spot(self,gme):
        in_spot = gme.is_car_in_spot(self)
        return in_spot

    async def failure(self,Game,send_response_channel):
        self.status = 'Failure'
        self.ref = []
        #if self.reserved:
        self.release_reserved_area(Game,send_response_channel)
        self.reserved = False
        # elif self in Game.reserved_areas_requested:
        #     Game.reserved_areas_requested.pop(self)
        await self.send_response(send_response_channel)

    async def run(self,send_response_channel,Game, Time, Logger):
        self.Logger = Logger
        self.Logger.info('{0} (ID {1}) - started'.format(self.name,self.id))
        #rospy.init_node('Robot_ctrl_{}'.format(self.car_nbr))
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command,send_response_channel,Game, Time)
            if self.cancel:
                print('Cancelling {0}'.format(self.name))
                nursery.cancel_scope.cancel()
            await trio.sleep(0)