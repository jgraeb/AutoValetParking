# End planner module
# Tung M. Phan
# California Institute of Technology
# March 5th, 2020
import copy as cp
import _pickle as pickle
from ipdb import set_trace as st
import matplotlib.pyplot as plt
import numpy as np
from ipdb import set_trace as st
import sys
sys.path.append("..")
from variables.global_vars import SCALE_FACTOR_PLAN as SFP

if __name__ == '__main__':
    from tools import (constrain_heading_to_pm_180, img_to_csv_bitmap,
    get_tube_for_lines, point_set_is_safe, compute_edge_weight,
    astar_trajectory, waypoints_to_headings, waypoints_to_curve, convert_to_edge_dict, segment_to_mpc_inputs,
    check_direction, pi_2_pi)
else:
    try:
        from motionplanning.tools import (constrain_heading_to_pm_180, img_to_csv_bitmap,
        get_tube_for_lines, point_set_is_safe, compute_edge_weight,
        astar_trajectory, waypoints_to_headings, convert_to_edge_dict, segment_to_mpc_inputs,
        check_direction, pi_2_pi)
    except:
        from tools import (constrain_heading_to_pm_180, img_to_csv_bitmap,
        get_tube_for_lines, point_set_is_safe, compute_edge_weight,
        astar_trajectory, waypoints_to_headings, convert_to_edge_dict, segment_to_mpc_inputs,
        check_direction, pi_2_pi)
import cv2
import sys
sys.path.append('..')
import components.game as game

def find_end_states_from_image(img_path):
    end_states = []
    img = cv2.imread('imglib/{}.png'.format(img_path))
    m = img.shape[0]
    n = img.shape[1]
    for i in range(m):
        for j in range(n):
            b,g,r = img[i][j] # format is BGR, positive integer types!
            if not (r == 255 and g == 255 and b == 255): # if not white color
                if r == 0:
                    if g == 0:
                        angle = -int(b) # need to recast b to an integer
                        end_states.append((j, i, angle, 0))
                        # reverse parking
#                        ell = 1
#                        i_r, j_r = j + ell * np.cos(angle / 180 * np.pi), i + ell * -np.sin(angle / 180 * np.pi)
#                        end_states.append((i_r, j_r, constrain_heading_to_pm_180(-angle), 0)) # need to recast b to an integer
                    elif b == 0:
                        angle = int(g)
                        end_states.append((j, i, angle, 0))
                        # reverse parking
#                        ell = 1
#                        i_r, j_r = j + ell * np.cos(angle / 180 * np.pi), i + ell * -np.sin(angle / 180 * np.pi)
#                        end_states.append((i_r, j_r, constrain_heading_to_pm_180(-angle), 0))
    print('found {} end states!'.format(len(end_states)))
    print(end_states)
    return end_states

class EndPlanner:
    def __init__(self, planning_graph, end_states, contract):
        self.graph = planning_graph['graph']
        self.edge_info = planning_graph['edge_info']
        self.end_states = end_states
        self.node_to_edge_map = planning_graph['node_to_edge_map']
        self.all_nodes = planning_graph['all_nodes']
        self.contract = contract
    def get_planning_graph(self):
        end_state_weight_penalty = 1000
        the_planning_graph = dict()
        for idx, end_state in enumerate(self.end_states):
            print('planning graph progress: {0:.1f}%'.format(idx/(len(self.end_states)-1)*100))
            for assume_state in self.graph._nodes:
                if self.contract.check_assumption(assume_state=assume_state, end_state=end_state):
                    node_sequence = self.contract.guart.generate_guarantee(assume_state, end_state)
                    edge = convert_to_edge_dict(assume_state, end_state, node_sequence)
                    self.graph.add_edges([[assume_state, end_state,
                        end_state_weight_penalty + compute_edge_weight(edge)]])
                    self.edge_info[assume_state, end_state] = node_sequence
                    for node in node_sequence:
                        if node in self.node_to_edge_map:
                            self.node_to_edge_map[node].add((assume_state, end_state))
                        else:
                            self.node_to_edge_map[node] = {(assume_state, end_state)}
                        self.all_nodes.add(node)
                    if assume_state[3] == 0: # assume state is stopping
                        # add reverse
                        reversed_node_sequence = node_sequence[::-1]
                        edge = convert_to_edge_dict(assume_state, end_state, reversed_node_sequence)
                        self.graph.add_edges([[end_state,
                            assume_state, end_state_weight_penalty + compute_edge_weight(edge)]])
                        self.edge_info[end_state, assume_state] = reversed_node_sequence
                        for node in reversed_node_sequence:
                            if node in self.node_to_edge_map:
                                self.node_to_edge_map[node].add((end_state, assume_state))
                            else:
                                self.node_to_edge_map[node] = {(end_state, assume_state)}
                            self.all_nodes.add(node)
        coverage = sum([int(end_state in self.graph._nodes) for end_state in self.end_states])/len(self.end_states)
        print('end state coverage is {0:.1f}%'.format(coverage*100))
        the_planning_graph['graph'] = self.graph
        the_planning_graph['edge_info'] = self.edge_info
        the_planning_graph['end_states'] = self.end_states
        the_planning_graph['node_to_edge_map'] = self.node_to_edge_map
        the_planning_graph['all_nodes'] = self.all_nodes
        return the_planning_graph

class EndStateContract:
    def __init__(self, assm, guart, bitmap):
        self.assm = assm
        self.guart = guart
        self.bitmap = bitmap.transpose()
    def check_assumption(self, assume_state, end_state):
        try:
            check = self.assm.check_assumption(assume_state, end_state)
            if check:
                node_sequence = self.guart.generate_guarantee(assume_state, end_state)
                tube = get_tube_for_lines(node_sequence, r=self.guart.uncertainty)
                return point_set_is_safe(tube, self.bitmap)
            else:
                return False
        except:
            return False
    def generate_guarantee(self, assume_state, end_state):
        return self.guart.generate_guarantee(assume_state, end_state)

class PrimitiveAssumption:
    def __init__(self, scan_radius):
        self.scan_radius = scan_radius
    def check_scan_radius_constraint(self, assume_state, end_state):
        assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
        end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
        dist = np.linalg.norm(assume_loc - end_loc)
        return dist <= self.scan_radius
    def check_assumption(self, assume_state, end_state):
        raise NotImplementedError

class PrimitiveGuarantee:
    def __init__(self):
        pass
    def generate_guarantee(self, assume_state, end_state):
        raise NotImplementedError

class TwoPointTurnAssumption(PrimitiveAssumption):
    def __init__(self, scan_radius, max_angle_diff, min_dist):
        super(TwoPointTurnAssumption, self).__init__(scan_radius)
        self.max_angle_diff = max_angle_diff
        self.min_dist = min_dist
    def check_assumption(self, assume_state, end_state):
        if self.check_scan_radius_constraint(assume_state, end_state):
            assume_angle = assume_state[2]
            end_angle = end_state[2]
            angle_diff = abs(constrain_heading_to_pm_180(assume_angle-end_angle))
            assume_angle = assume_angle / 180 * np.pi
            end_angle = end_angle / 180 * np.pi
            assume_dir = np.array([[np.cos(assume_angle),-np.sin(assume_angle)]]).transpose()
            end_dir = np.array([[np.cos(end_angle),-np.sin(end_angle)]]).transpose()
            dir_mat = np.hstack((assume_dir, -end_dir))
            assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
            end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
            intersect_params = np.matmul(np.linalg.inv(dir_mat), (end_loc - assume_loc))
            if not (intersect_params[0] >= 0 and intersect_params[1] <= 0):
                return False
            intersect = assume_loc + intersect_params[0] * assume_dir
            dist = np.linalg.norm(end_loc - intersect)
            return angle_diff <= self.max_angle_diff and dist >= self.min_dist
        else:
            return False

class TwoPointTurnGuarantee(PrimitiveGuarantee):
    def __init__(self, uncertainty):
        self.uncertainty = uncertainty
    def generate_guarantee(self, assume_state, end_state):
        assume_angle = assume_state[2]
        end_angle = end_state[2]
        angle_diff = abs(constrain_heading_to_pm_180(assume_angle-end_angle))
        assume_angle = assume_angle / 180 * np.pi
        end_angle = end_angle / 180 * np.pi
        assume_dir = np.array([[np.cos(assume_angle),-np.sin(assume_angle)]]).transpose()
        end_dir = np.array([[np.cos(end_angle),-np.sin(end_angle)]]).transpose()
        dir_mat = np.hstack((assume_dir, -end_dir))
        assume_loc = np.array([[assume_state[0], assume_state[1]]]).transpose()
        end_loc = np.array([[end_state[0], end_state[1]]]).transpose()
        intersect_params = np.matmul(np.linalg.inv(dir_mat), (end_loc - assume_loc))
        intersect = assume_loc + intersect_params[0] * assume_dir
        node_sequence = [(int(assume_loc[0]), int(assume_loc[1])),
                         (int(intersect[0]), int(intersect[1])),
                         (int(end_loc[0]), int(end_loc[1]))]
        return node_sequence


def update_planning_graph(planning_graph, del_nodes):
    new_planning_graph = cp.deepcopy(planning_graph)
    del_edges = []
    for node in del_nodes:
        if node in planning_graph['node_to_edge_map']:
            for edge in planning_graph['node_to_edge_map'][node]:
                del_edges.append((edge))
    for edge in del_edges:
        new_planning_graph['graph']._weights[edge] = np.inf
    return new_planning_graph

def get_mpc_path(start, end, planning_graph):
    edge_info_dict = planning_graph['edge_info']
    simple_graph = planning_graph['graph']
    traj, path_weight = astar_trajectory(simple_graph, start, end)
    all_segments = []
    for start, end in zip(traj, traj[1:]):
        all_segments.append(segment_to_mpc_inputs(start, end, edge_info_dict))
    return all_segments, path_weight

# check whether the path is blocked
# def subpath_is_safe(start, end):
#     is_safe = True
#     subpath = [[start[0],start[1]], [end[0],end[1]]]
#     for car in game.cars:
#         if distance([car.x,car.y],subpath) <= 10: # minimum allowed distance
#             is_safe = False
#             return is_safe
#     for ped in game.peds:
#         if distance([ped.state[0],ped.state[1]],subpath) <= 10: # minimum allowed distance
#             is_safe = False
#             return is_safe
#     return is_safe

def distance(a, b):
    b0 = b[0]
    b1 = b[1]
    return min(((a[0] - b0[0])**2 + (a[1] - b0[1])**2)**0.5,((a[0] - b1[0])**2 + (a[1] - b1[1])**2)**0.5)

def split_up_path(segments): # split the path up if there is a direction change
    directions = []
    split_here = []
    for segment in segments:
        direction = check_direction(segment)
        directions.append(direction)
    for idx,direction in enumerate(directions):
        if idx>=1 and direction!=directions[idx-1]:
            split_here.append(idx-1)
    subsegments = []
    startidx = 0
    for idx in split_here:
        subsegments.append(segments[startidx:idx+1])
        startidx = idx+1
    subsegments.append(segments[startidx:])
    return subsegments

def curvature_analysis(segment):
    # do curvature analysis
    dx_dt = np.gradient(segment[:, 0])
    dy_dt = np.gradient(segment[:, 1])
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5
    return curvature[:-1]

def check_curvature(segments):
    subsegments = split_up_path(segments)
    curvatures = []
    # smooth the path segments
    for subsegment in subsegments:
        waypoints = []
        for subsubsegment in subsegment:
            for waypoint in subsubsegment:
                waypoint = tuple(waypoint)
                if waypoint not in waypoints:
                    waypoints.append(waypoint)
        segment = np.array(waypoints_to_curve(waypoints))
        segment = segment*SFP
        curvature = curvature_analysis(segment)
        curvatures.append(curvature)
    print(curvatures)
    st()
    radii = [1/x if x!=0 else '999' for x in curvatures]
    print(radii)
    return curvatures

# def complete_path_is_safe(traj):
#     # check whether the subpath is safe
#     is_safe = True
#     for sub_start, sub_end in zip(traj, traj[1:]):
#         if not subpath_is_safe(sub_start, sub_end): # if someone blocks the subpath
#             is_safe = False
#             return is_safe
#     return is_safe

# def longest_safe_subpath(traj):
#     idx = -1
#     for sub_start, sub_end in zip(traj, traj[1:]):
#         idx += 1
#         if not subpath_is_safe(sub_start, sub_end):
#             safe_subpath = traj[0:idx]
#             safe_start = traj[idx]
#             return safe_subpath, safe_start

# TODO: make separate planner class
if __name__ == '__main__':
    remap = False
    smooth = True
    if remap:
        end_states = find_end_states_from_image('AVP_planning_300p_end_states')
        # end_states = find_end_states_from_image('AVP_planning_250p_end_states') # for ROS layout
        print(end_states)
        assume = TwoPointTurnAssumption(scan_radius=40,max_angle_diff=60,min_dist=15)
        guarantee = TwoPointTurnGuarantee(uncertainty=2)
        bitmap = img_to_csv_bitmap('AVP_planning_300p') # compute bitmap
        # bitmap = img_to_csv_bitmap('AVP_planning_250p_reachability') # compute bitmap for ROS layout
        contract = EndStateContract(assume, guarantee, bitmap)
        with open('planning_graph.pkl', 'rb') as f:
        # with open('planning_graph_250_reachability.pkl', 'rb') as f: # for ROS layout
            planning_graph = pickle.load(f)
        end_planner = EndPlanner(planning_graph=planning_graph,end_states=end_states,contract=contract)
        planning_graph = end_planner.get_planning_graph()
        with open('planning_graph_lanes.pkl', 'wb') as f:
            pickle.dump(planning_graph, f)
    else:
        with open('planning_graph_lanes.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        edge_info = planning_graph['edge_info']
        simple_graph = planning_graph['graph']
        ps = []
        # ps.append((60, 60, 0, 0)) # for ROS layout
        ps.append((120, 60, 0, 0))
        plt.plot(ps[0][0], ps[0][1], 'c.')
        img = plt.imread('imglib/AVP_planning_300p.png')
        # img = plt.imread('imglib/AVP_planning_250p.png') # for ROS layout
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
                        #print(traj)
                        print(weight)
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
                            print(segment)
                            plt.plot(segment[0,0], segment[0,1], 'b.')
                            plt.plot(segment[-1,0], segment[-1,1], 'rx')
                            plt.plot(segment[:,0], segment[:,1], 'k--')
                            plt.pause(0.1)
                            segments.append(segment)
                        if smooth:
                            curvature = check_curvature(segments)
                            # plt.plot(segment[0,0], segment[0,1], 'b.')
                            # plt.plot(segment[-1,0], segment[-1,1], 'rx')
                            # plt.plot(segment[:,0], segment[:,1], 'k--')
                            # plt.pause(0.1)
                        print('trajectory plotted!')
                        print('click to set desired xy')
                        clickok = True
                        plt.show()
                    except:
                        clickok = True
                        print('CANNOT FIND TRAJECTORY: click again to set xy!')
                        if len(ps) > 1:
                            ps = ps[:-1]
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
