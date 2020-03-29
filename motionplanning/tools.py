# Tung M. Phan
# California Institute of Technology
# February 10th, 2020
# Resolve cv2 conflict with ROS
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
import networkx as nx
from ipdb import set_trace as st

def img_to_csv_bitmap(img_path, save_name=None, verbose=False):
    # usage: img_to_bitmap(img) where img is a numpy array of RGB values with
    # drivable area masked as white
    # saves as 'save_name'.csv and then returns a bitmap of drivable area (1) for drivable, (0) otherwise
    img = cv2.imread('imglib/{}.png'.format(img_path))
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

def manhattan_distance(p1, p2):
    p1_xy = np.array([p1[0], p1[1]])
    p2_xy = np.array([p2[0], p2[1]])
    return np.sum(np.abs(p1_xy-p2_xy))

def get_rotation_matrix(theta, deg):
    if deg:
        theta = theta / 180 * np.pi
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def reflect_over_x_axis(vector):
    return np.array([vector[0], -vector[1]])

def constrain_heading_to_pm_180(heading):
    heading = heading % 360
    if heading > 180:
        heading = -(360-heading)
    return heading

def get_ball_neighbors(center, r):
    r = int(np.ceil(r)) # robustify r
    neighbors = []
    dy_max = r
    for dy in range(-dy_max, dy_max+1):
        dx_max = int(np.floor(np.sqrt(r**2-dy**2)))
        for dx in range(-dx_max, dx_max+1):
            neighbors.append([center[0]+dx, center[1]+dy])
    return np.unique(np.array(neighbors), axis=0)

def in_range(x, x_min, x_max):
    return x >= x_min and x <= x_max

def rotate_vector(vec, theta, deg=False):
    rot_mat = get_rotation_matrix(theta, deg=deg)
    outvec =  np.array([int(round(x)) for x in np.matmul(rot_mat, vec)])
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

def point_set_is_safe(point_set, bitmap):
    i_max = bitmap.shape[0]
    j_max = bitmap.shape[1]
    for point in point_set:
        if in_range(point[0], 0, i_max-1) and in_range(point[1], 0, j_max-1):
            if not bitmap[point[0]][point[1]]:
                return False
    return True

def waypoints_to_headings(waypoints, initial_heading):
    headings = []
    previous_heading = initial_heading
    for point1, point2 in zip(waypoints, waypoints[1:]):
        dys = point2[1] - point1[1]
        dxs = point2[0] - point1[0]
        new_heading = np.arctan2(-dys, dxs) / np.pi * 180
        if np.abs(constrain_heading_to_pm_180(new_heading-previous_heading)) >= 90:
            new_heading = constrain_heading_to_pm_180(new_heading + 180)
        headings.append(new_heading)
        previous_heading = new_heading
    headings.append(headings[-1])
    return headings

def compute_edge_weight(edge):
    weight = 0
    sequence = edge['node_sequence']
    start = edge['start_node']
    end = edge['end_node']
    dxs, dys = np.array(end[0:2]) - np.array(start[0:2])
    displacement_angle = np.arctan2(-dys, dxs) / np.pi * 180
    initial_heading = start[2]
    reversing = abs(constrain_heading_to_pm_180(displacement_angle - initial_heading)) >= 90
    reverse_penalty = 0
    if reversing:
        reverse_penalty = 10 # play with this
    velocity_change = abs(end[3] - start[3])
    headings = np.abs(waypoints_to_headings(sequence, initial_heading))
    heading_change = np.sum(headings)
    man_distance = 0
    for n1, n2 in zip(sequence, sequence[1:]):
        man_distance += manhattan_distance(n1, n2)
    weight = man_distance + heading_change * 0.1 + velocity_change + reverse_penalty
    return weight

def compute_sequence_weight(sequence):
    weight = 0
    for n1, n2 in zip(sequence, sequence[1:]):
        weight += manhattan_distance(n1, n2)
    return weight

def astar_trajectory(planning_graph,start,end,heuristic=None):
    closest_start = find_closest_point(start, planning_graph)
    closest_end = find_closest_point(end, planning_graph)
    nx_graph = convert_to_nx_graph(planning_graph)
    if heuristic:
        path = nx.astar_path(nx_graph, closest_start, closest_end, heuristic)
    else:
        path = nx.astar_path(nx_graph, closest_start, closest_end)
    weight = sum(nx_graph[u][v].get('weight', 1) for u, v in zip(path[:-1], path[1:]))
    path = np.array(path)
    return path, weight

def find_closest_point(p1, graph):
    def angle_similarity_scores(a_diff):
        c_diff = []
        for diff in a_diff:
            diff = constrain_heading_to_pm_180(diff)
            c_diff.append(diff)
        return np.array(c_diff)
    diff = np.array(graph._nodes)-p1
    if (diff.shape[1] == 4):
        return graph._nodes[np.argmin(np.sqrt(diff[:,0]**2 +
            diff[:,1]**2) + 0.001 * angle_similarity_scores(diff[:,2])**2 + 0.001 * diff[:,3]**2)]
    if (diff.shape[1] == 3):
        return graph._nodes[np.argmin(np.sqrt(diff[:,0]**2 +
            diff[:,1]**2 + 0.1 * diff[0:,2]**2))]
    if (diff.shape[1] == 2):
        return graph._nodes[np.argmin(np.sqrt(diff[:,0]**2 +
            diff[:,1]**2))]

def convert_to_nx_graph(digraph):
    G = nx.DiGraph()
    for start in digraph._edges:
        for end in digraph._edges[start]:
            edge = (start, end)
            weight = digraph._weights[edge]
            G.add_weighted_edges_from([(edge[0], edge[1], weight)])
    return G

def csv_bitmap_to_numpy_bitmap(file_name):
    with open('{}.csv'.format(file_name), 'rt') as f:
        np_bitmap = np.array(list(csv.reader(f, delimiter=','))).astype('bool')
    return np_bitmap

