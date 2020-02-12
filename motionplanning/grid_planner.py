# A-star algorithm and image processing
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
        np.savetxt(save_name + '.csv', np_bitmap, fmt='%i', delimiter=",")
    return np_bitmap

def csv_bitmap_to_numpy_bitmap(file_name):
    with open(file_name+'.csv', 'rt') as f:
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

def get_rotation_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def rotate_vector(vec, theta):
    rot_mat = get_rotation_matrix(theta)
    return np.array([int(round(x)) for x in np.matmul(rot_mat, vec)])

def get_rect_for_line(point1, point2, r):
    angle = np.arctan2(point2[1] - point1[1], point2[0] - point1[0])
    r = int(np.ceil(r)) # robustify r
    d = int(np.ceil(np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)))
    pre_displacements = []
    for dy in range(-r, r+1):
        for dx in range(0, d+1):
            pre_displacements.append([dx, dy])
    pre_displacements = np.matmul(np.array(pre_displacements), get_rotation_matrix(angle).transpose()).astype(int)
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

def get_grid_neighbors(point, sampled_points, grid_size):
    GridNeighbor = namedtuple('GridNeighbor', ['xy', 'weight'])
    neighbors = []
    for dx in [-grid_size, 0, grid_size]:
        for dy in [-grid_size, 0, grid_size]:
            weight = (np.abs(dx) + np.abs(dy))/grid_size
            neighbor_xy = tuple(np.array(point) + np.array([dx, dy]))
            if neighbor_xy in sampled_points: # check if neighbor_xy is in sampled_points:
                if neighbor_xy != point:
                    neighbors.append(GridNeighbor(xy=neighbor_xy, weight=weight))
    return neighbors

def grid_to_planning_graph(bitmap, grid, uncertainty, verbose=False):
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
        for neighbor in get_grid_neighbors(node, graph._nodes, grid.grid_size):
            neighbor_xy = neighbor.xy
            weight = neighbor.weight
            tube = get_tube_for_line(node, neighbor_xy, r=uncertainty)
            if point_set_is_safe(tube, bitmap):
                graph.add_double_edges([[node, neighbor_xy, weight]])
    return graph

def bitmap_to_planning_graph(np_bitmap, anchor, grid_size, uncertainty, planning_graph_save_name=None, verbose=True):
    grid = create_uniform_grid(np_bitmap, anchor = anchor, grid_size = grid_size)
    planning_graph = grid_to_planning_graph(bitmap=np_bitmap, grid=grid, uncertainty=uncertainty, verbose=verbose)
    if planning_graph_save_name:
        with open(planning_graph_save_name + '.pkl', 'wb') as f:
            pickle.dump(planning_graph, f)

def image_to_planning_graph(img_name, anchor, grid_size, uncertainty, planning_graph_save_name, verbose=True):
    np_bitmap = img_to_csv_bitmap(cv2.imread('imglib/{}.png'.format(img_name)), save_name=None, verbose=True)
    return bitmap_to_planning_graph(np_bitmap, anchor, grid_size, uncertainty, verbose, planning_graph_save_name)

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

def A_star(start, end, weighted_graph):
    node_score = namedtuple('node_score', ['x', 'y', 'score', 'parent'])
    def to_score_node(node, score, parent):
        return score_node(x=node[0], y=node[1], score=score, parent=parent)

    def get_manhattan_distance(p1, p2):
        return np.abs(p1.x - p2.x) + np.abs(p1.y - p2.y)

    def propagate_from(p1):
        for end in weighted_graph._edges[(p1.x, p1.y)]:
            pass

    assert start in weighted_graph._nodes and end in weighted_graph._nodes

    checked = [start]
    unchecked = weighted_graph._nodes
    x1 = to_score_node(start, score=None, parent=None)
    x2 = to_score_node(end, score=None, parent=None)
    print(get_manhattan_distance(x1, x2))
    print(propagate_from(x1))

    return shortest_path

if __name__ == '__main__':
#    image_to_planning_graph(img_name='AVP_planning_300p', planning_graph_save_name='planning_graph', anchor=[0,0], grid_size=10, uncertainty=5)
    planning_graph = open_pickle('planning_graph')
    start = planning_graph._nodes[23]
    end = planning_graph._nodes[223]
    print(A_star(start, end, planning_graph))


#    plt.gca().invert_yaxis()
#    plt.axis('equal')
#    plt.show()

