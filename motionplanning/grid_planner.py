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

def get_tube_for_lines(points, r):
    assert len(points) > 1
    tubes = []
    for point1, point2 in zip(points, points[1:]):
        tube = get_tube_for_line(point1, point2, r)
        tubes.append(tube)

    return np.unique(np.vstack(tubes), axis=0)

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

def get_car_successors(xy_heading, grid_size, weights=[1,3,3,4,5,5]):
    successors = []
    xy = xy_heading[0:2]
    heading = xy_heading[2]
    STEP = grid_size
    dstate = [[STEP, 0, 0], [STEP, STEP, 45], [STEP, -STEP, -45], [-STEP, 0, 0], [-STEP, -STEP, 45], [-STEP, STEP, -45]]
    for ds in dstate:
        dxy = np.array(ds[0:2])
        new_xy = np.array(xy) + reflect_over_x_axis(rotate_grid_displacement_vector(dxy, heading, deg=True))
        new_heading = constrain_heading_to_pm_180(heading + ds[2])
        successors.append((new_xy[0], new_xy[1], new_heading))
    return successors, weights

def get_car_neighbors(xy_heading, sampled_points, grid_size):
    CarNeighbor = namedtuple('CarNeighbor', ['xy', 'heading', 'weight'])
    neighbors = []
    xy = xy_heading[0:2]
    heading = xy_heading[2]
    successors, weights = get_car_successors(xy_heading=xy_heading, grid_size=grid_size)
    for idx, succ in enumerate(successors):
        new_x, new_y, new_heading = succ
        if (new_x, new_y, new_heading) in sampled_points: # TODO: quite inefficient, fix this
            new_xy = [new_x, new_y]
            if weights:
                weight = weights[idx]
                new_neighbor = CarNeighbor(xy=new_xy, heading=new_heading, weight=weight)
            else:
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


class Node:
    def __init__(self, x, y, heading, v):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v

def grid_to_prim_graph(bitmap, grid, uncertainty, verbose=False):
    ###################### TESTING HERE
    norm_grid_trajectory_set = [[[0, 0], [1, 0], [2, 0], [3, 0]],
                           [[0, 0], [1, 0], [2, -1], [3, -1]],
                           [[0, 0], [1, 0], [2, 1], [3, 1]],
                           [[0, 0], [1, 0], [2, -1], [2, -2]],
                           [[0, 0], [1, 0], [2, 1], [2, 2]]]

    sim_set = dict()
    sim_set[(0,0)] = [(2.730174518127961, 999.0, 0.5296237328741152), (3.1947179750584196, 999.0, 0.6039733519585504), (2.9600269611982193, 999.0, 0.6854793141852606), (2.946945375575089, 5.600000000000002, 0.5071482908799032), (2.9529104574295957, 999.0, 0.7179876356918411)]
    sim_set[(0,10)] = [(2.771060298574731, 4.800000000000002, 1.0150240121793508), (2.9958146609905723, 5.200000000000002, 1.187321100425719), (2.654892321503462, 5.000000000000002, 1.0783286235294025), (2.468691934613473, 5.000000000000002, 1.1684365223045305), (3.0712091487423554, 4.800000000000002, 1.1053176361027377)]
    sim_set[(10,0)] = [(2.770525158412713, 4.000000000000001, 0.43332719330960906), (3.397514451756727, 999.0, 0.5812877012246122), (3.275513793339767, 4.000000000000001, 0.5135220268209858), (3.1233673682088643, 3.800000000000001, 0.39062929765438076), (3.7439592048396695, 7.200000000000004, 0.44523762219328233)]
    sim_set[(10, 10)] = [(2.6356803875864845, 3.0000000000000004, 1.1170380912780782), (5.104216914134341, 3.2000000000000006, 2.133142862565424), (2.7332871495782167, 3.0000000000000004, 1.0311214654411611), (3.1020801322269205, 3.0000000000000004, 1.1900683266439613), (3.599854643056156, 2.8000000000000003, 1.1365753996863543)]

    grid_prim_set = GridPrimitiveSet(norm_grid_trajectory_set=norm_grid_trajectory_set, sim_set=sim_set, grid_size=10)
    #####################
    PrimitiveGraph = namedtuple('PrimitiveGraph', ['graph', 'edge_info'])
    bitmap = bitmap.transpose()
    graph = nx.DiGraph()
    edge_info = dict()
    sampled_points = grid.sampled_points
    all_nodes = []
    for point in sampled_points:
        neighbors = get_ball_neighbors(point, uncertainty)
        if point_set_is_safe(neighbors, bitmap):
            for heading in [0, 90, 180, -90]:
                for velocity in [0, 10]:
                    node = Node(x=point[0], y=point[1], heading=heading, v=velocity)
                    all_nodes.append(node)

    for idx, node in enumerate(all_nodes):
        if verbose:
            print('planning graph progress: {0:.1f}%'.format(idx/len(all_nodes)*100))
        for neighbor in grid_prim_set.get_neighbors(node):
            tube = get_tube_for_lines(neighbor.path, r=uncertainty)
            if point_set_is_safe(tube, bitmap):
                from_tuple = node.x, node.y, node.heading, node.v
                to_tuple = neighbor.node.x, neighbor.node.y, neighbor.node.heading, neighbor.node.v
                graph.add_edge(from_tuple, to_tuple)
                edge_info[from_tuple, to_tuple] = neighbor.path
                print(graph.edges)
                st()

    prim_graph = PrimitiveGraph(graph=graph, edge_info=edge_info)
    return prim_graph

class GridPrimitiveSet:
    def __init__(self, norm_grid_trajectory_set, sim_set, grid_size):
        self.grid_size = grid_size
        self.sim_set = sim_set
        self.grid_trajectory_set = (np.array(norm_grid_trajectory_set) * self.grid_size).tolist()
    def add_primitive(self, new_grid_trajectory):
        self.grid_trajectory_set.append((np.array(new_grid_trajectory) * self.grid_size).tolist())
    def get_neighbors(self, node):
        def get_final_heading_from_path(path):
            p1, p2 = path[-2:]
            dy = p2[1]-p1[1]
            dx = p2[0]-p1[0]
            heading = int(np.arctan2(-dx, dy) / np.pi * 180)
            return heading
        NodeNeighbor = namedtuple('NodeNeighbor', ['node', 'path'])
        heading = node.heading
        xy = np.array([node.x, node.y])
        neighbors = []
        for traj_idx, traj in enumerate(self.grid_trajectory_set):
            for velocity in [0, 10]:
                sim_res = self.sim_set[(node.v, velocity)][traj_idx]
                if sim_res[2] <= sim_res[0]:
                    path = []
                    for point in traj:
                        new_vec = reflect_over_x_axis(rotate_vector(point, heading, deg=True)) # TODO: pre-compute and store this as an attribute
                        new_vec = np.array(xy) + new_vec
                        path.append(new_vec.tolist())
                    final_heading = get_final_heading_from_path(path)
                    node_neighbor = NodeNeighbor(node=Node(x=path[-1][0],y=path[-1][1],heading=final_heading,v=velocity), path=path)
                    neighbors.append(node_neighbor)
        return neighbors

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

def bitmap_to_prim_graph(np_bitmap, anchor, grid_size, uncertainty, planning_graph_save_name=None, verbose=True):
    grid = create_uniform_grid(np_bitmap, anchor = anchor, grid_size = grid_size)
    planning_graph = grid_to_prim_graph(bitmap=np_bitmap, grid=grid, uncertainty=uncertainty, verbose=verbose)
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

def image_to_prim_graph(img_name, anchor, grid_size, uncertainty, planning_graph_save_name, verbose=True):
    np_bitmap = img_to_csv_bitmap(cv2.imread('imglib/{}.png'.format(img_name)), save_name=None, verbose=True)
    return bitmap_to_prim_graph(np_bitmap=np_bitmap, anchor=anchor, grid_size=grid_size, uncertainty=uncertainty, planning_graph_save_name=planning_graph_save_name, verbose=verbose)

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
    remap = True
    if remap:
        planning_graph = image_to_prim_graph(img_name='AVP_planning_300p',
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
        ps.append((200, 245, 0))
        ps.append((260, 60, 0))
#        ps.append((120, 55))
#        ps.append((100, 150))
#        ps.append((70, 215))
#        ps.append((207, 115))
#        ps.append((230, 60))
#        import time
#        start_time = time.time()
        for p in range(len(ps)-1):
            start = ps[p]
            end = ps[p+1]
            traj = astar_trajectory(planning_graph, start, end)
            plt.plot(traj[:,0], traj[:,1])
#        print("--- %s seconds ---" % (time.time() - start_time))
        print(traj)
        img = plt.imread('imglib/AVP_planning_300p.png')
        plt.imshow(img)
        plt.axis('equal')
        plt.show()
