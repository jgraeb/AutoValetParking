"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

modified for Automated Valet Parking (AVP)
"""
from scipy.spatial.distance import cdist
import random
from ipdb import set_trace as st
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
sys.path.append("../PathPlanning/CubicSpline/")
sys.path.append("..")
#sys.path.append("..")
#import grid_planner
#from data import parking_spots, exampletraj
from variables.global_vars import *
# try:
#     import cubic_spline_planner
# except:
#     raise
#mod
TOL = 0.5
BACK_TARGET_SPEED = -10.0 / 3.6  # [m/s] target speed
GOAL_SPEED = 0.0

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 3  # horizon length (5)

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 2.5  # goal distance
STOP_SPEED = 1.0 / 3.6  # stop speed
MAX_TIME = 200.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 7.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


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


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state. v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state. v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref, True)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    #else:
        #print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref, breaking=None):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    MAX_ACCEL = 1.0 #m/ss
    if breaking:
        MAX_ACCEL = 7.0 #m/ss emergency braking
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    MAX_ACCEL = 1.0 #m/ss
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind,goalspeed, last_segment):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    if not last_segment:
        isgoal = (d <= GOAL_DIS)
    else:
        isgoal = (d <= GOAL_DIS)
    if abs(tind - nind) >= 5:
        isgoal = False
    # modified
    delgoalspeed = abs(goalspeed)*1.5
    isstop = (abs(state.v) <= delgoalspeed)
    if (goalspeed == 0.0):
        #delgoalspeed == STOP_SPEED
        isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    if (goalspeed != 0.0):
        if isgoal:
            return True
    return False


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    vkmh = [state.v*3.6] # mod storing speed in km/h
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        state = update_state(state, ai, di)
        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)
        #vkmh.append(state.v*3.6) # mod storing speed in km/h
        if check_goal(state, goal, target_ind, len(cx),goalspeed,True): # modified goal speed
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)

    return t, x, y, yaw, v, d, a


def calc_speed_profile(cx, cy, cyaw, target_speed, goalvel, direction): # added direction

    speed_profile = [target_speed] * len(cx)
    #direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    # modified
    speed_profile[-1] = goalvel

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)
    print(cx2)
    print(cy2)
    print(cyaw2)
    return cx, cy, cyaw, ck

#  mod
def Trafo(X):
    X[:,0] = X[:,0]*2.5 # scaling factor 2.5 to make AVP dimensions and car dimensions match
    X[:,1] = X[:,1]*2.5
    return X

def check_direction(path):
    cx = path[:,0]*SCALE_FACTOR_PLAN
    cy = path[:,1]*SCALE_FACTOR_PLAN
    cyaw = np.deg2rad(path[:,2])
    dx = cx[1] - cx[0]
    dy = cy[1] - cy[0]
    move_direction = math.atan2(-dy, dx)
    # print(move_direction)
    # print(cyaw[0])
    dangle = abs(pi_2_pi(move_direction - cyaw[0]))
    if dangle >= math.pi / 4.0:
        direction = -1.0
    else:
        direction = 1.0
    #print('The direction is: '+str(direction))
    return direction

# split up trajectories further for simulation implementation
def get_course_2_dropoff(dl):
    # load the reference trajectory
    tr_ref_0 = np.loadtxt("paths/go2dropoff.txt")
    tr_ref= Trafo(tr_ref_0)
    cx, cy, cyaw = tr_ref[:,0], tr_ref[:,1],tr_ref[:,2]
    ck=0
    return cx, cy, cyaw, ck

def get_course_2_park(dl):
    # load the reference trajectory
    tr_ref_0 = np.loadtxt("paths/go2park.txt")
    tr_ref= Trafo(tr_ref_0)
    cx, cy, cyaw = tr_ref[:,0], tr_ref[:,1],tr_ref[:,2]
    ck=0
    return cx, cy, cyaw, ck

def get_course_2_pickup(dl):
    # load the reference trajectory
    tr_ref_0 = np.loadtxt("paths/go2pickup.txt")
    tr_ref= Trafo(tr_ref_0)
    cx, cy, cyaw = tr_ref[:,0], tr_ref[:,1],tr_ref[:,2]
    ck=0
    return cx, cy, cyaw, ck

def get_random_course(dl):
    ax = [random.random()*10, 0, 0]
    ay = [0, random.random()*10, 0]
    #print(ax)
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def get_error(pos,reftraj,t):
    err = np.zeros(len(pos))
    k=len(pos)
    for n in range(0,k):
        #print('position: '+str(pos[n]))#+str(row))
        errmatrix=[cdist([pos[n]], reftraj,'euclidean')]
        mindist = np.amin(errmatrix)
        #print('dist '+str(errmatrix))
        #print('mindist '+str(mindist))
        err[n]=mindist
    #print(err)
    maxerr = np.amax(err)#[:15])
    # find max error when passing though endpoint first time
    enderr = np.amin(err[-5:-1])#np.amin(err[-1])
    # find index at which err is less than tol
    try:
        idx = next(x for x, val in enumerate(err) if val < TOL)
        tmax=t[idx]
    except:
        tmax= 999
        #print(maxerr)
    return maxerr, tmax, enderr

def get_guarantee(x_d, dl):
    # set range of x_d
    max_err_t=[]
    for x in x_d:
        # generate random curves using cubic spline
        cx, cy, cyaw, ck = get_random_course(dl)
        sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,0.0,1)
        # Place car at x_d from traj start point
        # pick a state randomly in a circle around the starting point
        initial_state = get_random_initial_state(cx,cy,cyaw,x,0.0)#State(x=cx[0]+randx, y=cy[0]+randy, yaw=cyaw[0]+randyaw, v=0.0)
        # do simulation
        t, x, y, yaw, v, d, a, _ = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)
        # find max error
        cxa=np.asarray(cx)
        cya=np.asarray(cy)
        reftraj = np.stack((cxa, cya), axis=-1)#np.hstack((cxa,cya))
        xa=np.asarray(x)
        ya=np.asarray(y)
        pos = np.stack((xa, ya), axis=-1)#np.hstack((cxa,cya))
        # print(cxa)
        # print(cya)
        # print(reftraj)
        # only do beginning of trajectory, first 15 points
        tol = 0.25 # define tolerance
        max_err_t.append(get_error(pos[:15],reftraj,t))
        # return guarantee table with errors and time
    return max_err_t

def get_random_initial_state(x,y,yaw,x_d,startV):
        # Place car at x_d from traj start point
        # pick a state randomly in a circle around the starting point, do several and pick worst
        # pick random angle
        randangle = np.random.uniform()*2*np.pi
        randx = np.cos(randangle)*x_d
        randy = (x_d**2-randx**2)**0.5
        # random initial yaw should be correct direction +-45 degrees
        randyaw = np.random.uniform()*np.deg2rad(random.randrange(-45,45))
        #print(randyaw)
        initial_state = State(x=x+randx, y=y+randy, yaw=yaw+randyaw, v=startV)
        return initial_state

def get_grid_planner_prims(x_d,steps):
    # make grid primitives from starting position (0,0,0)
    # preallocate primitives
    prims = np.zeros((3**steps,steps+1,3))
    lastpos = np.array([(0,0,0)]) # define starting position at (0,0,0)
    prims[:,0,:] = lastpos
    #print(prims)
    #print(lastpos)
    for i in range(1,steps+1):
        #print('Step:'+str(i))
        rep = 3**steps/(3**(steps-i))
        num = int(3**steps/rep)
        #print('Num: '+str(num))
        next_step=np.zeros((lastpos.shape[0],3,3))
        for idxa in range(0,lastpos.shape[0]): # iterate though the number of last positions
            option=lastpos[idxa]
            #print('Position: '+str(option))
            next_step0,_ = grid_planner.get_car_successors(xy_heading=option, grid_size=x_d)
            #print(next_step0)
            next_step=np.array(next_step0[:3])
            #print('The next steps are:')
            #print(next_step)
            for idxb in range(0,3):#next_step.shape[1]): # iterate though new positions
                pos = next_step[idxb]
                #print('Pos: '+str(pos))
                if i==1:
                    for k in range(0,num):
                        #print('primindex: '+str((idxb*3)+k))
                        prims[(idxb*3)+k][i]= pos
                else:
                    for k in range(0,num):
                        #print('idxa:' +str(idxa))
                        #print('idxb:' +str(idxb))
                        #print('primindex: '+str((idxa*3)+idxb+k))
                        prims[(idxa*3)+idxb+k][i]= pos
                        #print(prims)

        lastpos=next_step
    #print('primitives:')
    #print(prims)
    # convert prims into mpc data format
    ck = 0
    cx = prims[:,:,0]
    cy = prims[:,:,1]
    cyaw = np.deg2rad(prims[:,:,2])
    #print(cyaw)
    print(cx)
    print(cy)
    cyaw[:,0] = cyaw[:,0]*(-1)
    cyaw[:,1] = cyaw[:,1]*(-1)
    cyaw[:,2] = cyaw[:,2]*(-1)
    print(cyaw)
    cx2=[[0,2,4]]
    cy2=[[0,0,-2]]
    cyaw2 = [[ 0.0 , -np.pi/8, -0.78539816]]
    return cx2, cy2, cyaw2, ck

def reverse_two_step(x_d): # reversing just one step
    # set target speed to negative
    start = [4,5,0]
    cx = np.zeros((1,2))
    cy = np.zeros((1,2))
    cyaw = np.zeros((1,2))
    cx = [[start[0] , start[0]-x_d, start[0]-2*x_d]]
    cy = [[start[1], start[1], start[1]]]
    cyaw = [[0,0,0]]
    ck = 0
    return cx, cy, cyaw, ck

def reverse_prims(x_d): # reversing just one step
    # set target speed to negative
    start = [4,5,0]
    cx = np.zeros((3,2))
    cy = np.zeros((3,2))
    cyaw = np.zeros((3,2))
    next_step0,_ = grid_planner.get_car_successors(xy_heading=start, grid_size=x_d)
    back_step=np.array(next_step0[3:])
    #print(back_step)
    #print(back_step.shape)
    cx[:,0] = start[0]
    cx[0,1] = back_step[0,0]
    cx[1,1] = back_step[1,0]
    cx[2,1] = back_step[2,0]
    cy[:,0] = start[1]
    cy[0,1] = back_step[0,1]
    cy[1,1] = back_step[1,1]
    cy[2,1] = back_step[2,1]
    cyaw[0,0] = start[2]
    cyaw[0,1] = np.deg2rad(back_step[0,2])*(-1)
    cyaw[1,1] = np.deg2rad(back_step[1,2])*(-1)
    cyaw[2,1] = np.deg2rad(back_step[2,2])*(-1)
    #cyaw = [i - math.pi for i in cyaw]
    ck = 0
    # get the correct headings for the first points
    cyaw2 = get_heading(cx,cy)
    # print(cx)
    # print(cy)
    cyaw2 = [i-np.pi for i in cyaw2]
    #print(cyaw)
    #print(cyaw2)
    return cx[0], cy[0], cyaw2[0], ck

def switchback_turn(x_d):
    step = x_d
    cx = np.zeros((1,4))
    cy = np.zeros((1,4))
    cyaw = np.zeros((1,4))
    cx[0,0] = 0
    cx[0,1] = cx[0,0]+step
    cx[0,2] = cx[0,1]-step
    cx[0,3] = cx[0,2]+step
    cy[0,0] = 0
    cy[0,1] = cy[0,0]-step
    cy[0,2] = cy[0,1]
    cx[0,3] = cx[0,2]+step
    cyaw[0,0] = -1/4*np.pi
    cyaw[0,1] = 0
    cyaw[0,2] = 1/4*np.pi
    cyaw[0,3] = 1/4*np.pi
    ck = 0
    print(cx)
    print(cy)
    print(cyaw)
    return cx,cy,cyaw,ck

def get_grid_prims(x_d):
    # get cx,cy,cyaw from grid planner
    cx = np.zeros((5,4))
    cy=np.zeros((5,4))
    cyaw=np.zeros((5,4))
    # Start at
    startx = 4
    starty = 3
    # set up x coordinates
    cx[:,0] = startx
    cx[:,1]= cx[0,0]+x_d
    cx[:,2]= cx[0,1]+x_d
    cx[:,3] = cx[0,2]+x_d
    cx[3,3] = cx[0,2]
    cx[4,3] = cx[0,2]
    # set up y coordinates
    cy[:,:] = starty
    cy[1,2] = cy[1,0]-x_d
    cy[1,3] = cy[1,2]
    cy[2,2] = cy[2,1]+x_d
    cy[2,3] = cy[2,2]
    cy[3,2] = cy[3,1] - x_d
    cy[3,3] = cy[3,2] - x_d
    cy[4,2] = cy[4,1] + x_d
    cy[4,3] = cy[4,2] + x_d
    # Now calculate yaw to be directly towards the next point
    cyaw[:,:] = 0
    cyaw[1,1] = -1/4*np.pi
    cyaw[2,1] = 1/4*np.pi
    cyaw[3,1] = -1/4*np.pi
    cyaw[4,1] = 1/4*np.pi
    cyaw[3,2] = -1/2*np.pi
    cyaw[4,2] = 1/2*np.pi
    cyaw[3,3] = -1/2*np.pi
    cyaw[4,3] = 1/2*np.pi
    ck = 0
    # print(cx)
    # print(cy)
    #(cyaw)

    return cx, cy, cyaw, ck#


def get_heading(x,y): # find heading towards the next point from x,y positions for MPC
    heading = np.zeros(np.shape(x))
    for i, xvals in enumerate(x):
        #print('i= '+str(i))
        for k in range(0,(len(xvals))):
            #print('k = '+str(k))
            if k<(len(xvals)-1):
                dely = y[i,k+1]-y[i,k]
                delx = xvals[k+1]-xvals[k]
                heading[i,k] = math.atan2(dely,delx)
            else:
                heading[i,k] = heading[i,k-1] # keep the same heading for last point
    return heading

async def track_async(cx, cy, cyaw, ck, sp, dl, initial_state,goalspeed):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    vkmh = [state.v*3.6] # mod storing speed in km/h
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        state = update_state(state, ai, di)
        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)
        #vkmh.append(state.v*3.6) # mod storing speed in km/h

        if check_goal(state, goal, target_ind, len(cx),goalspeed): # modified goal speed
            print("Goal")
            break

    return t, x, y, yaw, v, d, a

def guarantee_reverse(x_d,dl):
    num_sim = 50

    max_err_0_0 = []
    max_err_0_V = []
    max_err_V_0 = []
    max_err_V_V = []

    REVERSE_TARGET_SPEED = TARGET_SPEED/2
    for x in x_d:
        # generate the motion primitives using simple grid
        cx_vec, cy_vec, cyaw_vec, ck = reverse_two_step(x)
        # max distance is half of the diagonal in a grid
        maxx=1/np.sqrt(2)*x
        for n,val in enumerate(cx_vec):
            # calculate speed profile
            cx=cx_vec[n]
            cy=cy_vec[n]
            cyaw=cyaw_vec[n]
            V_vec =[0.0, - REVERSE_TARGET_SPEED]
            V_goal_vec = [0.0, - REVERSE_TARGET_SPEED]
            maxerr = np.zeros(num_sim)
            #max_err = []
            maxerrend = np.zeros(num_sim)
            headingdiff = np.zeros(num_sim)
            vel_err = np.zeros(num_sim)
            for i, vel in enumerate(V_vec):
                for n,goalvel in enumerate(V_goal_vec):
                    sp = calc_speed_profile(cx, cy, cyaw, REVERSE_TARGET_SPEED, goalvel, direction=-1)
                    initial_heading = cyaw[0]
                    for k in range(0,num_sim): # simulate for k different initial conditions
                        initial_state = get_random_initial_state(cx[0],cy[0],initial_heading,maxx, vel)
                        # do simulation
                        t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state, goalvel)
                        # find max error
                        cxa=np.asarray(cx)
                        cya=np.asarray(cy)
                        reftraj = np.stack((cxa, cya), axis=-1)#np.hstack((cxa,cya))
                        xa=np.asarray(x)
                        ya=np.asarray(y)
                        pos = np.stack((xa, ya), axis=-1)#np.hstack((cxa,cya))
                        # only do beginning of trajectory, first 15 points
                        #tol = 0.25 # define tolerance
                        headingdiff[k] = abs(cyaw[-1]-yaw[-1])
                        # find error at the end of trajectory
                        maxerr[k],_,maxerrend[k] = get_error(pos,reftraj,t)
                        vel_err[k] = (v[-1]-goalvel)*3.6
                        #max_err.append((np.amax(maxerr),np.amax(maxerrend), np.rad2deg(np.amax(headingdiff))))
                    if (i==0) and (n==0):
                        max_err_0_0.append((np.amax(maxerr),np.amax(maxerrend),np.rad2deg(np.amax(headingdiff)),np.amax(vel_err)))
                    elif (i==0) and (n==1):
                        max_err_0_V.append((np.amax(maxerr),np.amax(maxerrend),np.rad2deg(np.amax(headingdiff)),np.amax(vel_err)))
                    elif (i==1) and (n==0):
                        max_err_V_0.append((np.amax(maxerr),np.amax(maxerrend),np.rad2deg(np.amax(headingdiff)),np.amax(vel_err)))
                    elif (i==1) and (n==1):
                        max_err_V_V.append((np.amax(maxerr),np.amax(maxerrend),np.rad2deg(np.amax(headingdiff)),np.amax(vel_err)))
    return max_err_0_0, max_err_0_V, max_err_V_0, max_err_V_V

def guarantee_for_motionprims(x_d,dl):
    # set range of x_d
    #max_err_t=[]
    max_err_0_0 = []
    max_err_0_V = []
    max_err_V_0 = []
    max_err_V_V = []
    num_sim = 50
    for x in x_d:
        # generate the motion primitives using simple grid
        cx_vec, cy_vec, cyaw_vec, ck = get_grid_prims(x)
        #cx_vec, cy_vec, cyaw_vec, ck = switchback_turn(x)
        #cx_vec, cy_vec, cyaw_vec, ck = reverse(x)
        # max distance is half of the diagonal in a grid
        maxx=1/np.sqrt(2)*x
        # cx_vec = [cx_vec[1]]
        # cy_vec = [cy_vec[1]]
        # cyaw_vec = [cyaw_vec[1]]
        # test heading function
        #heading = get_heading(cx_vec,cy_vec)
        # loop over all 5 motion primitives
        for n,val in enumerate(cx_vec):
            cx=cx_vec[n]
            cy=cy_vec[n]
            cyaw=cyaw_vec[n]
            # Place car at max. distance from traj start point
            # do the simulation for ~10 different initial states and choose the maximum error values
            # Loop over V
            V_vec =[0.0,TARGET_SPEED]
            V_goal_vec = [0.0,TARGET_SPEED]
            #V = V_vec[1]
            maxerr = np.zeros(num_sim)
            errend = np.zeros(num_sim)
            terr = np.zeros(num_sim)
            yaws_err = np.zeros(num_sim)
            vel_err = np.zeros(num_sim)
            #maxerr_v = np.zeros((2,3))
            for i,vel in enumerate(V_vec):
                for n,goalvel in enumerate(V_goal_vec):
                    # calculate speed profile
                    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED, goalvel,1)
                    for k in range(0,num_sim): # simulate for k different initial conditions
                        initial_state = get_random_initial_state(cx[0],cy[0],cyaw[0],maxx,vel)
                        # do simulation
                        t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state,goalvel)
                        # find max error
                        cxa=np.asarray(cx)
                        cya=np.asarray(cy)
                        reftraj = np.stack((cxa, cya), axis=-1)#np.hstack((cxa,cya))
                        xa=np.asarray(x)
                        ya=np.asarray(y)
                        pos = np.stack((xa, ya), axis=-1)#np.hstack((cxa,cya))
                        # only do beginning of trajectory, first 15 points
                        tol = 0.25 # define tolerance
                        # save final heading error
                        yaws_err[k] = abs(yaw[-1]-cyaw[-1])
                        # save final velocity error
                        vel_err[k] = (v[-1]- goalvel)*3.6
                        # find error at the end of trajectory
                        maxerr[k], terr[k], errend[k] = get_error(pos,reftraj,t)
                    # find max values and save them
                    #maxerr_v[i] = (np.amax(maxerr),np.amax(terr),np.amax(errend))
                    if (i==0) and (n==0):
                        max_err_0_0.append((np.amax(maxerr),np.amax(terr),np.amax(errend),np.rad2deg(np.amax(yaws_err)),np.amax(vel_err)))
                    elif (i==0) and (n==1):
                        max_err_0_V.append((np.amax(maxerr),np.amax(terr),np.amax(errend),np.rad2deg(np.amax(yaws_err)),np.amax(vel_err)))
                    elif (i==1) and (n==0):
                        max_err_V_0.append((np.amax(maxerr),np.amax(terr),np.amax(errend),np.rad2deg(np.amax(yaws_err)),np.amax(vel_err)))
                    elif (i==1) and (n==1):
                        max_err_V_V.append((np.amax(maxerr),np.amax(terr),np.amax(errend),np.rad2deg(np.amax(yaws_err)),np.amax(vel_err)))
            #max_err_t.append((np.amax(maxerr_v[:,0]), np.amax(maxerr_v[:,1]), np.amax(maxerr_v[:,2])))#,np.amax(terr),np.amax(errend)))
            #print(max_err_t)
        # return guarantee table with max errors and time and uncertainty at the end
    return max_err_0_0, max_err_0_V, max_err_V_0, max_err_V_V


def main():
    print(__file__ + " start!!")
    dl = 1.0  # course tick
    #cx, cy, cyaw, ck = get_course_2_park(dl)
    cx, cy, cyaw, ck = get_switch_back_course(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,0.0,1)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state,0.0)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="Reference Trajectory")
        plt.plot(x, y, "-g", label="Tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        # mod
        plt.subplots()
        plt.plot(t, vkmh, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")

        plt.show()

def main_guarantee_reverse():
    # grid planner size size is in pixels - convert to meters
    # 1 pixels are 0.306 ~ 0.3 meters
    x_pix = [5]
    x_d=[val*0.3 for val in x_pix]
    print('Grid Size = '+str(x_d)+' m / '+str(x_pix)+'pixels')
    dl = 1.0  # course tick

    max_err_0_0, max_err_0_V, max_err_V_0, max_err_V_V = guarantee_reverse(x_d, dl)
    print('[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(0,0)] = '+ str(max_err_0_0))
    print('[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(0,V)] = '+ str(max_err_0_V))
    print('[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(V,0)] = '+ str(max_err_V_0))
    print('[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(V,V)] = '+ str(max_err_V_V))

    f= open("guarantee.txt","w+")
    f.write("[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(0,0)] = "+ str(max_err_0_0)+"\r\n")
    f.write("[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(0,V)] = "+ str(max_err_0_V)+"\r\n")
    f.write("[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(V,0)] = "+ str(max_err_V_0)+"\r\n")
    f.write("[Max. Error, Error @ Endpoint, Heading [deg], Velocity Error [km/h] | vel=(V,V)] = "+ str(max_err_V_V)+"\r\n")
    f.close()

def main_guarantee():
    print(__file__ + " start!!")
    # grid planner size size is in pixels - convert to meters
    # 1 pixels are 0.306 ~ 0.3 meters
    x_pix = [10]#, 15]
    x_d=[val*0.3 for val in x_pix]
    print('Grid Size = '+str(x_d)+' m / '+str(x_pix)+'pixels')
    dl = 1.0  # course tick
    max_err_0_0, max_err_0_V, max_err_V_0, max_err_V_V = guarantee_for_motionprims(x_d, dl)
    print('[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(0,0)] = '+ str(max_err_0_0))
    print('[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(0,V)] = '+ str(max_err_0_V))
    print('[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(V,0)] = '+ str(max_err_V_0))
    print('[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(V,V)] = '+ str(max_err_V_V))

    f= open("guarantee_50sim_15pix.txt","w+")
    f.write("[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(0,0)] = "+ str(max_err_0_0)+"\r\n")
    f.write("[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(0,V)] = "+ str(max_err_0_V)+"\r\n")
    f.write("[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(V,0)] = "+ str(max_err_V_0)+"\r\n")
    f.write("[Max. Error, Time, Position Error @ Endpoint, Heading Error [deg], Velocity Error [km/h] | vel=(V,V)] = "+ str(max_err_V_V)+"\r\n")
    f.close()


def stop_car(path,startv): # bringing car to a full stop asap
    #if not self.status == 'Stop' or not self.status=='Blocked' or not self.status=='Conflict':
    # bring car to a full stop
    #print('STOPPING CAR {0}, velocity {1}'.format(self.id, self.v))
    odelta, oa = None, None
    #buffer = startv / 10 * 0.4
    print(path)
    cx = path[:,0]
    print(cx)
    cy = path[:,1]
    cyaw = path[:,2]
    print(cyaw)
    ck = 0
    dl = 1.0
    sp = [startv, 0,0]
    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=startv)
    t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="Reference Trajectory")
        plt.plot(x, y, "--g", label="Tracking")

        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        # mod
        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")

        plt.show()

    # target_ind, _ = tracking.calc_nearest_index(self.state, cx, cy, cyaw, 0)
    # sp = [self.v/2, self.v/4, 0]
    # xref, target_ind, dref = tracking.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, dl, target_ind)
    # x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
    # st()
    # oa, odelta = await self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
    # if odelta is not None:
    #     di, ai = odelta[0], oa[0]
    # self.state = tracking.update_state(self.state, ai, di)
    # st()
    # self.x = self.state.x
    # self.y = self.state.y
    # self.yaw = self.state.yaw
    # self.v = self.state.v
    # self.status == 'Stop'
    # #self.v = 0


def track_path_forward(ref):
    dl = 1
    ck = 0
    #cx, cy, cyaw, ck = get_switch_back_course(dl)
    # cx = ref[:,0]*SCALE_FACTOR_PLAN
    # cx = cx.reshape(len(ref),1)
    # cy = ref[:,1]*SCALE_FACTOR_PLAN
    # cy= cy.reshape(len(ref),1)
    # cyaw = np.deg2rad(ref[:,2]).reshape(len(ref),1)
    # direction = check_direction(ref)
    print(ref)
    for i in range(0,len(ref)-1):
        path = ref[i:i+2][:]
        print(path)
        cx = path[:,0]
        print(cx)
        cy = path[:,1]
        cyaw = path[:,2]
        #state = np.array([self.x, self.y,self.yaw])
        #  check  direction of the segment
        direction = check_direction(path)
        print("Direction is "+str(direction))
        sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED,0.0,direction)
        initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
        t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state,0.0)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="Reference Trajectory")
        plt.plot(x, y, "-g", label="Tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        # mod
        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")

        plt.show()

def test(path):
    check_direction(path)

if __name__ == '__main__':
    #main()
    #main_guarantee()

    x = 0
    y = 0
    yaw = np.pi/4


    direction = 1
    startv = 7/3.6
    buffer = startv * 3.6 / 10 * 0.4 * 2
    path = np.array([[ x, y, yaw,   startv],[ x + 0.5 * direction*buffer*np.cos(yaw), y + 0.5 * direction*buffer*np.sin(yaw), yaw,   startv/2],[ x + direction*buffer*np.cos(yaw), y + direction*buffer*np.sin(yaw), yaw,  0]])

    direc = [[x, y, yaw]]
    direc.append([x-direction*5*np.cos(yaw), y-direction*5*np.sin(yaw), yaw])
    direc.append([x-direction*10*np.cos(yaw), y-direction*10*np.sin(yaw), yaw])
    directive = np.array(direc)
    #st()
    #print(path)
    #print('buffer'+str(buffer))
    print(directive)
    #stop_car(path,startv)
    track_path_forward(directive)

    #path = np.array([[140.,  60.,   90.],[140.,  70.,   90.]])
    #path = np.array([[  40.,  140.,  -90.],[  40.,  150., -135.]])
    #test(path)
