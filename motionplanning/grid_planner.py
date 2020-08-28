# Grid planner module, image processing, and A-star path finding
# Tung M. Phan
# California Institute of Technology
# February 10th, 2020

from typing import List, Any, Tuple
import itertools
from collections import namedtuple
from planning_graph import WeightedDirectedGraph
import csv
import numpy as np
import matplotlib.pyplot as plt
import _pickle as pickle
import json
from collections import OrderedDict as od
from ipdb import set_trace as st
from prim_json_maker import import_json
from tools import rotate_vector, reflect_over_x_axis, constrain_heading_to_pm_180, img_to_csv_bitmap, get_tube_for_lines, get_ball_neighbors, in_range, point_set_is_safe, compute_edge_weight, astar_trajectory

class Node:
    def __init__(self, x, y, heading, v):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v

class GridPrimitiveSet:
    def __init__(self, json_dict, symmetrize=True):
        if symmetrize:
            all_sets = [self.symmetrize_entry(json_dict[key]) for key in json_dict]
            self.grid_trajectory_set = list(itertools.chain(*all_sets))
        else:
            self.grid_trajectory_set = [self.add_heading_and_rescale_node_sequence(json_dict[key]) for key in json_dict]

    def rotate_trajectory(self, traj, angle, deg):
        rotated_entry = od()
        path = []
        for point in traj:
            new_vec = reflect_over_x_axis(rotate_vector(point, angle, deg=deg))
            path.append(new_vec.tolist())
        return path

    def get_headings_from_path(self, path, travel_direction):
        if travel_direction == 'forward':
            p1s, p2s = path[:2]
            dys = p2s[1]-p1s[1]
            dxs = p2s[0]-p1s[0]
            p1e, p2e = path[-2:]
            dye = p2e[1]-p1e[1]
            dxe = p2e[0]-p1e[0]
            start_heading = int(np.arctan2(-dys, dxs) / np.pi * 180)
            end_heading = int(np.arctan2(-dye, dxe) / np.pi * 180)
        else:
            start_heading, end_heading = self.get_headings_from_path(path, 'forward')
            start_heading = constrain_heading_to_pm_180(start_heading + 180)
            end_heading = constrain_heading_to_pm_180(end_heading + 180)
        return start_heading, end_heading

    def get_edges_for_xy(self, xy):
        edges = []
        for prim in self.grid_trajectory_set:
            edge = dict()
            node_sequence = [(point[0]+xy[0], point[1]+xy[1]) for point in prim['node_sequence']]
            start_node = (node_sequence[0][0], node_sequence[0][1], prim['start_heading'], prim['start_v'])
            end_node = (node_sequence[-1][0], node_sequence[-1][1], prim['end_heading'], prim['end_v'])
            edge['node_sequence'] = node_sequence
            edge['start_node'] = start_node
            edge['end_node'] = end_node
            edges.append(edge)
        return edges

    def add_heading_and_rescale_node_sequence(self, entry):
        traj = entry['node_sequence']
        start_heading, end_heading = self.get_headings_from_path(path=entry['node_sequence'],
                                                         travel_direction=entry['prim_type'])
        new_traj = [[point[0] * entry['grid_size'], point[1] * entry['grid_size']] for point in traj]
        entry['start_heading'] = start_heading
        entry['end_heading'] = end_heading
        entry['node_sequence'] = new_traj
        return entry

    def symmetrize_entry(self, entry):
        traj = entry['node_sequence']
        new_entries = [self.add_heading_and_rescale_node_sequence(entry)]
        angles = [-90, 90, 180] # assuming square grid
        for angle in angles:
            new_traj = self.rotate_trajectory(traj, angle, deg=True)
            new_entry = dict(entry) # make copy of original entry
            new_entry['node_sequence'] = new_traj
            new_entries.append(self.add_heading_and_rescale_node_sequence(new_entry))
        return new_entries


def json_to_grid_primitive_set(infile):
    json_dict = import_json(infile)
    prim_set = GridPrimitiveSet(json_dict)
    return prim_set

class GridParams:
    def __init__(self, grid_size, grid_anchor):
        self.grid_size = grid_size
        self.grid_anchor = grid_anchor

class GridPlanner:
    def __init__(self, bitmap, prim_set, grid_params, uncertainty):
        self.bitmap = bitmap
        self.grid_size = grid_params.grid_size
        self.grid_anchor = grid_params.grid_anchor
        self.grid = self.create_uniform_grid()
        self.prim_set = prim_set
        self.uncertainty = uncertainty
        self.planning_graph = None

    def create_uniform_grid(self):
        bitmap = self.bitmap
        grid_size = self.grid_size
        grid_anchor = self.grid_anchor
        Grid = namedtuple('Grid', ['sampled_points', 'grid_size'])
        h = bitmap.shape[0]
        w = bitmap.shape[1]
        assert grid_anchor[0] >= 0 and grid_anchor[0] <= w # check range x
        assert grid_anchor[1] >= 0 and grid_anchor[1] <= h # check range y
        x_start = grid_anchor[0] % grid_size
        y_start = grid_anchor[1] % grid_size
        sampled_points = []
        x_curr = x_start
        y_curr = y_start
        while x_curr < w:
            y_curr = y_start
            while y_curr < h:
                sampled_points.append([x_curr, y_curr])
                y_curr += grid_size
            x_curr += grid_size
        grid = Grid(sampled_points=np.array(sampled_points), grid_size = grid_size)
        return grid

    def get_planning_graph(self, verbose=True):
        bitmap = self.bitmap.transpose()
        if not self.planning_graph:
            planning_graph = dict()
            planning_graph['graph'] = None
            planning_graph['edge_info'] = None
            graph = WeightedDirectedGraph()
            edge_info = dict()
            node_to_edge_map = dict()
            all_nodes = set()
            sampled_points = self.grid.sampled_points
            all_xy_nodes = []
            for xy in sampled_points:
                neighbors = get_ball_neighbors(xy, self.uncertainty)
                if point_set_is_safe(neighbors, bitmap):
                    all_xy_nodes.append(xy)

            for idx, xy in enumerate(all_xy_nodes):
                if verbose:
                    print('planning graph progress: {0:.1f}%'.format(idx/len(all_xy_nodes)*100))
                for edge in self.prim_set.get_edges_for_xy(xy):
                    tube = get_tube_for_lines(edge['node_sequence'], r=self.uncertainty)
                    if point_set_is_safe(tube, bitmap):
                        graph.add_edges([[edge['start_node'], edge['end_node'], compute_edge_weight(edge)]])
                        edge_info[edge['start_node'], edge['end_node']] = edge['node_sequence']
                        for node in edge['node_sequence']:
                            if node in node_to_edge_map:
                                node_to_edge_map[node].add((edge['start_node'], edge['end_node']))
                            else:
                                node_to_edge_map[node] = {(edge['start_node'], edge['end_node'])}
                            all_nodes.add(node)

            planning_graph['graph'] = graph
            planning_graph['edge_info'] = edge_info
            planning_graph['node_to_edge_map'] = node_to_edge_map
            planning_graph['all_nodes'] = all_nodes
            self.planning_graph = planning_graph
            return planning_graph
        else:
            return self.planning_graph

def plot_planning_graph(planning_graph, plt, verbose=True):
    # very inefficient plotting
    edges = [] # type: List[Any]
    for idx, edge in enumerate(planning_graph._edges):
        print('collecting graph edges progress: {0:.1f}%'.format(idx/len(planning_graph._edges)*100))
        for to_edge in planning_graph._edges[edge]:
            if [edge, to_edge] not in edges and [edge, to_edge] not in edges:
                edges.append([edge, to_edge])
    for idx, edge in enumerate(edges):
        print('plotting graph edges progress: {0:.1f}%'.format(idx/len(edges)*100))
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]])

if __name__ == '__main__':
    remap = True
    if remap:
        # create bitmap from parking lot image
        bitmap = img_to_csv_bitmap('AVP_planning_250p') # compute bitmap
        # define grid parameters
        grid_params = GridParams(grid_size = 10, grid_anchor = [0, 0])
        # load primitive set
        prim_set = json_to_grid_primitive_set('10px_prims_hacked.json')
        grid_planner = GridPlanner(bitmap, prim_set, grid_params, uncertainty = 7)
        planning_graph = grid_planner.get_planning_graph()
        with open('planning_graph.pkl', 'wb') as f:
            pickle.dump(planning_graph, f)
    else:
        with open('planning_graph.pkl', 'rb') as f:
            planning_graph = pickle.load(f)
        edge_info = planning_graph['edge_info']
        planning_graph = planning_graph['graph']
        ps = []
        ps.append((120, 60, 0, 0))
        plt.plot(ps[0][0], ps[0][1], 'c.')
        img = plt.imread('imglib/AVP_planning_250p.png')
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
                        traj, _ = astar_trajectory(planning_graph, start, end)
                        for start, end in zip(traj, traj[1:]):
                            segment = np.array(edge_info[(tuple(start), tuple(end))])
                            plt.plot(segment[0,0], segment[0,1], 'b.')
                            plt.plot(segment[-1,0], segment[-1,1], 'rx')
                            plt.plot(segment[:,0], segment[:,1], 'k--')
                            plt.pause(0.0001)
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
