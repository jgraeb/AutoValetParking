# Image processing and A-star path finding
# Tung M. Phan
# California Institute of Technology
# February 10th, 2020

import networkx as nx
from typing import List, Any, Tuple
from collections import namedtuple
from planning_graph import WeightedDirectedGraph
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from ipdb import set_trace as st
import _pickle as pickle
import heapq

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

def rotate_vector(vec, theta, deg=False):
    rot_mat = get_rotation_matrix(theta, deg=deg)
    outvec =  np.array([int(round(x)) for x in np.matmul(rot_mat, vec)])
    return outvec

def rotate_grid_displacement_vector(vec, theta, deg=False):
    scale = np.max(np.abs(vec))
    vec = vec/scale
    outvec = rotate_vector(vec, theta, deg) * scale
    return outvec

def get_rect_for_line(point1, point2, r):
    angle = np.arctan2(point2[1] - point1[1], point2[0] - point1[0])
    r = int(np.ceil(r)) # robustify r
    d = int(np.ceil(np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)))
    pre_displacements = []
    for dy in range(-r, r+1):
        for dx in range(0, d+1):
            pre_displacements.append([dx, dy])
    pre_displacements = np.matmul(np.array(pre_displacements), get_rotation_matrix(angle, deg=False).transpose()).astype(int)
    offset = np.tile(np.array(point1), (pre_displacements.shape[0], 1))
    displacements = pre_displacements + offset
    return np.unique(displacements, axis=0)

def get_tube_for_line(point1, point2, r):
    ball1 = get_ball_neighbors(point1, r)
    ball2 = get_ball_neighbors(point2, r)
    rect = get_rect_for_line(point1, point2, r)
    tube = np.vstack((ball1, ball2, rect))
    return np.unique(tube, axis=0)

def in_range(x, x_min, x_max):
    return x >= x_min and x <= x_max

def point_set_is_safe(point_set, bitmap):
    i_max = bitmap.shape[0]
    j_max = bitmap.shape[1]
    for point in point_set:
        if in_range(point[0], 0, i_max-1) and in_range(point[1], 0, j_max-1):
            if not bitmap[point[0]][point[1]]:
                return False
    return True

def get_pacman_neighbors(point, sampled_points, grid_size):
    PacmanNeighbor = namedtuple('PacmanNeighbor', ['xy', 'weight'])
    neighbors = []
    for dx in [-grid_size, 0, grid_size]:
        for dy in [-grid_size, 0, grid_size]:
            weight = (np.abs(dx) + np.abs(dy))/grid_size
            neighbor_xy = tuple(np.array(point) + np.array([dx, dy]))
            if neighbor_xy in sampled_points: # check if neighbor_xy is in sampled_points:
                if neighbor_xy != point:
                    neighbors.append(PacmanNeighbor(xy=neighbor_xy, weight=weight))
    return neighbors

def constrain_heading_to_pm_180(heading):
    heading = heading % 360
    if heading > 180:
        heading = -(360-heading)
    return heading

def reflect_over_x_axis(vector):
    return np.array([vector[0], -vector[1]])

def get_car_neighbors(xy_heading, sampled_points, grid_size, reflect_over_y = True):
    CarNeighbor = namedtuple('CarNeighbor', ['xy', 'heading', 'weight'])
    neighbors = []
    xy = xy_heading[0:2]
    heading = xy_heading[2]
    STEP = grid_size
    # x, y, heading, weight
    dstate = [[STEP, 0, 0, 1], [-STEP, 0, 0, 3], [STEP, STEP, 45, 5], [STEP, -STEP, -45, 5], [-STEP, STEP, -45, 4], [-STEP, -STEP, 45, 4]]
#    dstate = [[STEP, 0, 0, 1], [-STEP, 0, 0, 3], [STEP, STEP, 45, 2], [STEP, -STEP, -45, 2], [-STEP, STEP, -45, 4], [-STEP, -STEP, 45, 4]]
    for ds in dstate:
        dxy = np.array(ds[0:2])
        new_xy = np.array(xy) + reflect_over_x_axis(rotate_grid_displacement_vector(dxy, heading, deg=True))
        new_heading = constrain_heading_to_pm_180(heading + ds[2])
        if (new_xy[0], new_xy[1], new_heading) in sampled_points: # TODO: quite inefficient, fix this
            weight = ds[3]
            new_neighbor = CarNeighbor(xy=new_xy, heading=new_heading, weight=weight)
            neighbors.append(new_neighbor)
    return neighbors

def grid_to_pacman_graph(bitmap, grid, uncertainty, verbose=False):
    bitmap = bitmap.transpose()
    graph = WeightedDirectedGraph()
    sampled_points = grid.sampled_points
    for point in sampled_points:
        neighbors = get_ball_neighbors(point, uncertainty)
        if point_set_is_safe(neighbors, bitmap):
            graph.add_node(tuple(point))
    for idx, node in enumerate(graph._nodes):
        if verbose:
            print('planning graph progress: {0:.1f}%'.format(idx/len(graph._nodes)*100))
        for neighbor in get_pacman_neighbors(node, graph._nodes, grid.grid_size):
            neighbor_xy = neighbor.xy
            weight = neighbor.weight
            tube = get_tube_for_line(node, neighbor_xy, r=uncertainty)
            if point_set_is_safe(tube, bitmap):
                graph.add_double_edges([[node, neighbor_xy, weight]])
    return graph

def grid_to_car_graph(bitmap, grid, uncertainty, verbose=False):
    bitmap = bitmap.transpose()
    graph = WeightedDirectedGraph()
    sampled_points = grid.sampled_points
    for point in sampled_points:
        neighbors = get_ball_neighbors(point, uncertainty)
        if point_set_is_safe(neighbors, bitmap):
            for heading in [0, 45, 90, 135, 180, -45, -90, -135]:
                graph.add_node((point[0], point[1], heading))
    for idx, node in enumerate(graph._nodes):
        if verbose:
            print('planning graph progress: {0:.1f}%'.format(idx/len(graph._nodes)*100))
        for neighbor in get_car_neighbors(node, graph._nodes, grid.grid_size):
            neighbor_xy = neighbor.xy
            node_xy = [node[0], node[1]]
            weight = neighbor.weight
            tube = get_tube_for_line(node_xy, neighbor_xy, r=uncertainty)
            if point_set_is_safe(tube, bitmap):
                graph.add_edges([[node, (neighbor_xy[0], neighbor_xy[1], neighbor.heading), weight]])
    return graph

def bitmap_to_pacman_graph(np_bitmap, anchor, grid_size, uncertainty, planning_graph_save_name=None, verbose=True):
    grid = create_uniform_grid(np_bitmap, anchor = anchor, grid_size = grid_size)
    planning_graph = grid_to_pacman_graph(bitmap=np_bitmap, grid=grid, uncertainty=uncertainty, verbose=verbose)
    if planning_graph_save_name:
        with open('{}.pkl'.format(planning_graph_save_name), 'wb') as f:
            pickle.dump(planning_graph, f)
    return planning_graph

def bitmap_to_car_graph(np_bitmap, anchor, grid_size, uncertainty, planning_graph_save_name=None, verbose=True):
    grid = create_uniform_grid(np_bitmap, anchor = anchor, grid_size = grid_size)
    planning_graph = grid_to_car_graph(bitmap=np_bitmap, grid=grid, uncertainty=uncertainty, verbose=verbose)
    if planning_graph_save_name:
        with open('{}.pkl'.format(planning_graph_save_name), 'wb') as f:
            pickle.dump(planning_graph, f)
    return planning_graph

def image_to_pacman_graph(img_name, anchor, grid_size, uncertainty, planning_graph_save_name, verbose=True):
    np_bitmap = img_to_csv_bitmap(cv2.imread('imglib/{}.png'.format(img_name)), save_name=None, verbose=True)
    return bitmap_to_pacman_graph(np_bitmap=np_bitmap, anchor=anchor, grid_size=grid_size, uncertainty=uncertainty, planning_graph_save_name=planning_graph_save_name, verbose=verbose)

def image_to_car_graph(img_name, anchor, grid_size, uncertainty, planning_graph_save_name, verbose=True):
    np_bitmap = img_to_csv_bitmap(cv2.imread('imglib/{}.png'.format(img_name)), save_name=None, verbose=True)
    return bitmap_to_car_graph(np_bitmap=np_bitmap, anchor=anchor, grid_size=grid_size, uncertainty=uncertainty, planning_graph_save_name=planning_graph_save_name, verbose=verbose)

def open_pickle(file_name):
    with open('{}.pkl'.format(file_name), 'rb') as f:
        content = pickle.load(f)
    return content

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

def convert_to_nx_graph(digraph):
    G = nx.DiGraph()
    for start in digraph._edges:
        for end in digraph._edges[start]:
            edge = (start, end)
            weight = digraph._weights[edge]
            G.add_weighted_edges_from([(edge[0], edge[1], weight)])
    return G

def manhattan_distance(p1, p2):
    p1_xy = np.array([p1[0], p1[1]])
    p2_xy = np.array([p2[0], p2[1]])
    return np.sum(np.abs(p1_xy-p2_xy))

def find_closest_point(p1, graph):
    diff = np.array(graph._nodes)-p1
    if (diff.shape[1] == 3):
        return graph._nodes[np.argmin(np.sqrt(diff[:,0]**2 +
            diff[:,1]**2 + 0.1 * diff[0:,2]**2))]
    if (diff.shape[1] == 2):
        return graph._nodes[np.argmin(np.sqrt(diff[:,0]**2 +
            diff[:,1]**2))]

def astar_trajectory(planning_graph, start, end, heuristic=None):
    closest_start = find_closest_point(start, planning_graph)
    closest_end = find_closest_point(end, planning_graph)
    nx_graph = convert_to_nx_graph(planning_graph)
    if heuristic:
        path = np.array(nx.astar_path(nx_graph, closest_start, closest_end, heuristic))
    else:
        path = np.array(nx.astar_path(nx_graph, closest_start, closest_end))
    return path

if __name__ == '__main__':
    remap = False
    if remap:
        planning_graph = image_to_car_graph(img_name='AVP_planning_300p',
                planning_graph_save_name='planning_graph',
                anchor=[0,0], grid_size=10, uncertainty=7)
        img = plt.imread('imglib/AVP_planning_300p.png')
        plt.imshow(img)
        if len(planning_graph._nodes) <= 1000: # if not too many
            for node in planning_graph._nodes:
                plt.plot(node[0], node[1], '.')
            plt.axis('equal')
            plt.show()
    else:
        planning_graph = open_pickle('planning_graph')
        ps = []
        ps.append((120, 60, 0))
        ps.append((170, 92, -90))
        ps.append((80, 150, 180))
        ps.append((144, 245, 0))
#        ps.append((120, 55))
#        ps.append((100, 150))
#        ps.append((70, 215))
#        ps.append((207, 115))
#        ps.append((230, 60))
        for p in range(len(ps)-1):
            start = ps[p]
            end = ps[p+1]
            traj = astar_trajectory(planning_graph, start, end)
            plt.plot(traj[:,0], traj[:,1])
        print(traj)
        img = plt.imread('imglib/AVP_planning_300p.png')
        plt.imshow(img)
        plt.axis('equal')
        plt.show()
