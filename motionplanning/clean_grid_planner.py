# Image processing and A-star path finding
# Tung M. Phan
# California Institute of Technology
# February 10th, 2020

# Resolve cv2 conflict with ROS
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import networkx as nx
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

def img_to_csv_bitmap(img, save_name=None, verbose=False):
    # usage: img_to_bitmap(img) where img is a numpy array of RGB values with
    # drivable area masked as white
    # saves as 'save_name'.csv and then returns a bitmap of drivable area (1) for drivable, (0) otherwise
    m = img.shape[0]
    n = img.shape[1]
    np_bitmap = np.zeros((m,n), dtype=bool)
    for i in range(m):
        for j in range(n):
            np_bitmap[i][j] = all(img[i][j] == [255, 255, 255]) # if white
        if verbose:
            print('bitmap progress: {0:.1f}%'.format((i*n+j)/(m*n)*100))
    if save_name:
        np.savetxt('{}.csv'.format(save_name), np_bitmap, fmt='%i', delimiter=",")
    return np_bitmap

def csv_bitmap_to_numpy_bitmap(file_name):
    with open('{}.csv'.format(file_name), 'rt') as f:
        np_bitmap = np.array(list(csv.reader(f, delimiter=','))).astype('bool')
    return np_bitmap

def create_uniform_grid(np_bitmap, anchor, grid_size):
    Grid = namedtuple('Grid', ['sampled_points', 'grid_size'])
    h = np_bitmap.shape[0]
    w = np_bitmap.shape[1]
    assert anchor[0] >= 0 and anchor[0] <= w # check range x
    assert anchor[1] >= 0 and anchor[1] <= h # check range y
    x_start = anchor[0] % grid_size
    y_start = anchor[1] % grid_size
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

def get_ball_neighbors(center, r):
    r = int(np.ceil(r)) # robustify r
    neighbors = []
    dy_max = r
    for dy in range(-dy_max, dy_max+1):
        dx_max = int(np.floor(np.sqrt(r**2-dy**2)))
        for dx in range(-dx_max, dx_max+1):
            neighbors.append([center[0]+dx, center[1]+dy])
    return np.unique(np.array(neighbors), axis=0)

def get_rotation_matrix(theta, deg):
    if deg:
        theta = theta / 180 * np.pi
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def constrain_heading_to_pm_180(heading):
    heading = heading % 360
    if heading > 180:
        heading = -(360-heading)
    return heading

def reflect_over_x_axis(vector):
    return np.array([vector[0], -vector[1]])

def rotate_vector(vec, theta, deg=False):
    rot_mat = get_rotation_matrix(theta, deg=deg)
    outvec =  np.array([int(round(x)) for x in np.matmul(rot_mat, vec)])
    return outvec

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
            self.grid_trajectory_set = [json_dict[key] for key in json_dict]

    def rotate_trajectory(self, traj, angle, deg):
        rotated_entry = od()
        path = []
        for point in traj:
            new_vec = reflect_over_x_axis(rotate_vector(point, angle, deg=deg))
            path.append(new_vec.tolist())
        return path

    def symmetrize_entry(self, entry):
        traj = entry['node_sequence']
        new_entries = [entry]
        angles = [-90, 90, 180] # assuming square grid
        for angle in angles:
            new_traj = self.rotate_trajectory(traj, angle, deg=True)
            new_entry = dict(entry) # make copy of original entry
            new_entry['node_sequence'] = new_traj
            new_entries.append(new_entry)
        return new_entries


#    def get_neighbors(self, node):
#        def get_final_heading_from_path(path):
#            p1, p2 = path[-2:]
#            dy = p2[1]-p1[1]
#            dx = p2[0]-p1[0]
#            heading = int(np.arctan2(-dy, dx) / np.pi * 180)
#            return heading
#        NodeNeighbor = namedtuple('NodeNeighbor', ['node', 'path'])
#        heading = node.heading
#        xy = np.array([node.x, node.y])
#        neighbors = []
#        for traj_idx, traj in enumerate(self.grid_trajectory_set):
#            for velocity in [0, 10]:
#                sim_res = self.sim_set[(node.v, velocity)][traj_idx]
#                if sim_res[2] <= sim_res[0]:
#                    path = []
#                    for point in traj:
#                        new_vec = reflect_over_x_axis(rotate_vector(point, heading, deg=True)) # TODO: pre-compute and store this as an attribute
#                        new_vec = np.array(xy) + new_vec
#                        path.append(new_vec.tolist())
#                    final_heading = get_final_heading_from_path(path)
#                    node_neighbor = NodeNeighbor(node=Node(x=path[-1][0],y=path[-1][1],heading=final_heading,v=velocity), path=path)
#                    neighbors.append(node_neighbor)
#        return neighbors

def json_to_grid_primitive_set(infile):
    json_dict = import_json(infile)
    prim_set = GridPrimitiveSet(json_dict)
    return prim_set

prim_set = json_to_grid_primitive_set('10px_prims.json')
st()
