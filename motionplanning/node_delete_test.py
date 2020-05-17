# Tung M. Phan
# California Institute of Technology
# March 5th, 2020
import _pickle as pickle
import matplotlib.pyplot as plt
import numpy as np
from ipdb import set_trace as st
from tools import astar_trajectory, segment_to_mpc_inputs, get_nodes_to_delete
import traceback
import logging
import end_planner
import scipy.interpolate as interpolate
import itertools

def waypoints_to_curve(waypoints):
    if len(waypoints) > 4:
        t = [0]
        arc_length = 0
        for n1, n2 in zip(waypoints, waypoints[1:]):
            arc_length += np.sqrt((n1[0]-n2[0])**2 + (n1[1]-n2[1])**2)
            t.append(arc_length)
        x = np.array([point[0] for point in waypoints])
        y = np.array([point[1] for point in waypoints])
        # s for smoothness, k for degree
        tx, cx, kx = interpolate.splrep(t, x, s=20, k=4)
        ty, cy, ky = interpolate.splrep(t, y, s=20, k=4)
        spline_x = interpolate.BSpline(tx, cx, kx, extrapolate=False)
        spline_y = interpolate.BSpline(ty, cy, ky, extrapolate=False)
        return list(zip(spline_x(t), spline_y(t)))
    else:
        return waypoints



with open('planning_graph_lanes.pkl', 'rb') as f:
    planning_graph = pickle.load(f)
#NODES_TO_DELETE = [(170, 90), (170, 100), (210, 100), (200, 150), (90, 150)]
NODES_TO_DELETE = get_nodes_to_delete(planning_graph, [170, 182], 20)
planning_graph = end_planner.update_planning_graph(planning_graph, NODES_TO_DELETE)
edge_info = planning_graph['edge_info']
simple_graph = planning_graph['graph']
ps = []
# plot obstacles:
for NODE in NODES_TO_DELETE:
    plt.plot(NODE[0], NODE[1], 'ro')
#starting_at = [170.,  70., -90, 0]
starting_at = [120, 60, 0, 0]
ps.append(starting_at)
plt.plot(ps[0][0], ps[0][1], 'c.')
img = plt.imread('imglib/AVP_planning_300p.png')
fig = plt.figure(1)
plt.imshow(img)
plt.axis('equal')
coords = []
clicks = 0
print('click on parking lot to set next desired xy')
clickok = True
def onclick(event):
    global ix, iy, clicks, coords, ps, clickok
    if clickok:
        clickok = False
        ix, iy = event.xdata, event.ydata
        clicks += 1
        coords.append((ix, iy))
        if clicks % 2: # if odd
            print('x = %d, y = %d'%( ix, iy))
            print('click on another point to set desired heading')
            clickok = True
        else:
            try:
                dys = coords[1][1] - coords[0][1]
                dxs = coords[1][0] - coords[0][0]
                theta = np.arctan2(-dys, dxs) / np.pi * 180
                print('theta = %d'%(theta))
                ps.append((coords[0][0], coords[0][1], theta, 0))
                coords = []
                start = ps[-2]
                end = ps[-1]
                traj, weight = astar_trajectory(simple_graph, start, end)
                if weight == np.inf:
                    raise ValueError('PATH WEIGHT IS INFINITE!')
                #print(traj)
                print('THE PATH WEIGHT IS ' + str(weight))
                # while not complete_path_is_safe(traj):
                #     safe_subpath, safe_start = longest_safe_subpath(traj)
                #      # TODO: not sure how to generate the path
                #     new_subpath = astar_trajectory(simple_graph, safe_start, end)
                #     traj = safe_subpath + new_subpath
                segments = []
                for start, end in zip(traj, traj[1:]):
                    #print('Start'+str(start))
                    #print(end)
                    segment = segment_to_mpc_inputs(start, end, edge_info)
                    segments.append(segment)

                waypoints = []
                for segment in segments:
                    for waypoint in segment:
                        waypoint = tuple(waypoint)
                        if waypoint not in waypoints:
                            waypoints.append(waypoint)
                segment = np.array(waypoints_to_curve(waypoints))

                plt.plot(segment[0,0], segment[0,1], 'b.')
                plt.plot(segment[-1,0], segment[-1,1], 'rx')
                plt.plot(segment[:,0], segment[:,1], 'k--')
                plt.pause(0.1)
                print('trajectory plotted!')
                print('click to set desired xy')
                clickok = True
                plt.show()
            except Exception:
                logging.error(traceback.format_exc())
                clickok = True
                print('CANNOT FIND TRAJECTORY: click again to set xy!')
                if len(ps) > 1:
                    ps = ps[:-1]
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()
