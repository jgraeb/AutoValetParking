from components.boxcomponent import BoxComponent
import trio
from variables.global_vars import *
import motiontracking.mpc_tracking as tracking
import math
from components.game import Game

class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

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

    async def update_planner_command(self,send_response_channel,Game):
        async for directive in self.in_channels['Planner']:
            self.ref = directive
            print('{0} - Receiving Directive from Planner'.format(self.name))
            await self.track_reference(self.ref,Game,send_response_channel)
            await trio.sleep(0)
            await self.send_response(send_response_channel)

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

    async def track_async(self, cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed,Game,direction): # modified from MPC
        goal = [cx[-1], cy[-1]]
        self.state = initial_state
        # initial yaw compensation
        if self.state.yaw - cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0
        time = 0.0
        # x = [self.state.x]
        # y = [self.state.y]
        # yaw = [self.state.yaw]
        # v = [self.state.v]
        # t = [0.0]
        # d = [0.0]
        # a = [0.0]
        target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
        odelta, oa = None, None
        cyaw = tracking.smooth_yaw(cyaw)
        while tracking.MAX_TIME >= time:
            while not await self.check_path(Game,direction):
                self.status = 'Stop'
                # check for conflict
                await self.check_conflict(Game,direction)
                await trio.sleep(3) 
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
            if tracking.check_goal(self.state, goal, target_ind, len(cx),goalspeed): # modified goal speed
                break
 
    async def track_reference(self,ref,Game,send_response_channel):
        self.status = 'Driving'
        print('{0} - Tracking reference...'.format(self.name))
        ck = 0 
        dl = 1.0  # course tick
        #print(ref)
        # including a failure
        failidx = 5#np.random.randint(low=0, high=len(ref)-1, size=1)
        #print(failidx)
        for i in range(0,len(ref)-1):
            if (i==failidx):
                print('{0} Failing'.format(self.name))
                await self.failure(send_response_channel)
                break  
            self.status = 'Driving'
            path = ref[:][i]
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            state = np.array([self.x, self.y,self.yaw])
            #  check  direction of the segment
            direction = tracking.check_direction(path)
            #print("Direction is "+str(direction))
            # while not await self.check_path(Game,direction):
            #     self.status = 'Stop'
            #     # check for conflict
            #     await self.check_conflict(Game,direction)
            #     await trio.sleep(3)   
                # check for conflict, if conflict stop all execution and send to planner - supervisor
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,TARGET_SPEED,direction)
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,TARGET_SPEED,Game,direction)
            await trio.sleep(0)
        if not self.status == 'Failure':
            state = np.array([self.x, self.y,self.yaw])
            path = ref[:][-1]
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            direction = tracking.check_direction(path)
            while not await self.check_path(Game,direction):
                self.status = 'Stop'
                await self.check_conflict(Game,direction)
                await trio.sleep(3)
            initial_state = State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,0.0,direction)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)
            self.status = 'Completed'

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = self.status
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def check_path(self, gme, direction):
        #print('Checking the path')
        is_clear = await gme.check_car_path(self, direction)
        return is_clear

    async def check_conflict(self,gme,direction): # finish this
        #is_conflict = await gme.check_car_conflicts(self, direction)
        is_conflict = False
        return is_conflict

    async def stop(self,send_response_channel):
        self.status = 'Stop'
        await self.send_response(send_response_channel)

    async def failure(self,send_response_channel):
        self.status = 'Failure'
        await self.send_response(send_response_channel)
        await trio.sleep(1000) # freeze

    async def run(self,send_response_channel,Game):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command,send_response_channel,Game)
            await trio.sleep(0)