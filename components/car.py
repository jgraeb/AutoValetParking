from components.boxcomponent import BoxComponent
import trio
from variables.global_vars import *
import motiontracking.mpc_tracking as tracking
import math

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

    async def update_planner_command(self,send_response_channel):
        async for directive in self.in_channels['Planner']:
            self.ref = directive
            print('{0} - Receiving Directive from Planner'.format(self.name))
            await self.track_reference(self.ref)
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

    async def track_async(self, cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed): # modified from MPC
        goal = [cx[-1], cy[-1]]
        tracking.state = initial_state
        # initial yaw compensation
        if tracking.state.yaw - cyaw[0] >= math.pi:
            tracking.state.yaw -= math.pi * 2.0
        elif tracking.state.yaw - cyaw[0] <= -math.pi:
            tracking.state.yaw += math.pi * 2.0
        time = 0.0
        x = [tracking.state.x]
        y = [tracking.state.y]
        yaw = [tracking.state.yaw]
        v = [tracking.state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = tracking.calc_nearest_index(tracking.state, cx, cy, cyaw, 0)
        odelta, oa = None, None
        cyaw = tracking.smooth_yaw(cyaw)
        while tracking.MAX_TIME >= time:
            xref, target_ind, dref = tracking.calc_ref_trajectory(tracking.state, cx, cy, cyaw, ck, sp, dl, target_ind)
            x0 = [tracking.state.x, tracking.state.y, tracking.state.v, tracking.state.yaw]  # current state
            oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
            if odelta is not None:
                di, ai = odelta[0], oa[0]
            tracking.state = tracking.update_state(tracking.state, ai, di)
            time = time + tracking.DT
            self.x = tracking.state.x
            self.y = tracking.state.y
            self.yaw = tracking.state.yaw
            self.v = tracking.state.v
            await trio.sleep(0)
            if tracking.check_goal(tracking.state, goal, target_ind, len(cx),goalspeed): # modified goal speed
                break
 
    async def track_reference(self,ref):
        print('{0} - Tracking reference...'.format(self.name))
        ck = 0 
        dl = 1.0  # course tick
        for i in range(0,len(ref)-1):
            #print('Going to'+str(ref[i,:]))
            path = ref[:][i]
            cx = path[:,0]*SCALE_FACTOR_PLAN
            cy = path[:,1]*SCALE_FACTOR_PLAN
            cyaw = np.deg2rad(path[:,2])*-1
            state = np.array([self.x, self.y,self.yaw])
            sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,TARGET_SPEED,1)
            initial_state = tracking.State(x=state[0], y=state[1], yaw=state[2], v=self.v)
            await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,TARGET_SPEED)
            await trio.sleep(0)
        state = np.array([self.x, self.y,self.yaw])
        path = ref[:][-1]
        cx = path[:,0]*SCALE_FACTOR_PLAN
        cy = path[:,1]*SCALE_FACTOR_PLAN
        cyaw = np.deg2rad(path[:,2])*-1
        initial_state = tracking.State(x=state[0], y=state[1], yaw=state[2], v=self.v)
        sp = tracking.calc_speed_profile(cx, cy, cyaw, TARGET_SPEED/2,0.0,1)
        await self.track_async(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)

    async def send_response(self,send_response_channel):
        await trio.sleep(1)
        response = 'Completed'
        print('{0} - sending {1} response to Planner'.format(self.name,response))
        await send_response_channel.send((self,response))
        await trio.sleep(1)

    async def run(self,send_response_channel):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.update_planner_command,send_response_channel)
            await trio.sleep(0)