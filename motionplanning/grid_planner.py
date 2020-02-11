# A-star algorithm and image processing
# Tung M. Phan
# California Institute of Technology
# February 10th, 2020

from planning_graph import DirectedGraph
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from ipdb import set_trace as st

def img_to_csv_bitmap(img, save_name, verbose=False):
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
            print('progress: {0:.1f}'.format((i*n+j)/(m*n)*100))
    np.savetxt('{}.csv'.format(save_name), np_bitmap, fmt='%i', delimiter=",")
    return np_bitmap

def csv_bitmap_to_numpy_bitmap(file_name):
    with open('{}.csv'.format(file_name), 'rt') as f:
        np_bitmap = np.array(list(csv.reader(f, delimiter=','))).astype('bool')
    return np_bitmap

def uniform_sample_grid_points(np_bitmap, anchor, grid_size):
    h = np_bitmap.shape[0]
    w = np_bitmap.shape[1]
    assert anchor[0] >= 0 and anchor[0] <= w # check range x
    assert anchor[1] >= 0 and anchor[1] <= h # check range y
    x_start = anchor[0] % grid_size
    y_start = anchor[1] % grid_size
    sample_points = []
    x_curr = x_start
    y_curr = y_start
    while x_curr < w:
        y_curr = y_start
        while y_curr < h:
            sample_points.append([x_curr, y_curr])
            y_curr += grid_size
        x_curr += grid_size
    return np.array(sample_points)

#img_to_csv_bitmap(cv2.imread('imglib/AVP_planning_300p.png'), 'bitmap', verbose=True)

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

def to_planning_graph(bitmap, sampled_points, uncertainty):
    bitmap = bitmap.transpose()
    graph = DirectedGraph()
    for point in sampled_points:
        neighbors = get_ball_neighbors(point, uncertainty)
        if point_set_is_safe(neighbors, bitmap):
            graph.add_node(point)
    return np.array(graph._nodes)

def A_star(start, end, weighted_graph):
    def get_manhattan_distance(start, end):
        return np.abs(start.x - end.x) + np.abs(start.y - end.y)

    checked = []
    unchecked = []

if __name__ == '__main__':
    np_bitmap = csv_bitmap_to_numpy_bitmap('bitmap')
    anchor = [0, 0]
    grid_size = 5
    uncertainty = 2
    sampled = uniform_sample_grid_points(np_bitmap, anchor = anchor, grid_size = grid_size)

    planning_graph = to_planning_graph(bitmap=np_bitmap, sampled_points=sampled, uncertainty=uncertainty)

#    for i in range(100):
#        point1 = [30, 20]
#        point2 = [55, 80]
#        point3 = [100, 100]
#        r = 10
#        tube1 = get_tube_for_line(point1, point2, r)
#        tube2 = get_tube_for_line(point2, point3, r)
#
    plt.gca().invert_yaxis()
    plt.plot(planning_graph[:,0], planning_graph[:,1], '.')
#    plt.plot(sampled[:,0], sampled[:,1], '.')
#    plt.plot(tube1[:,0], tube1[:,1], 'r.')
#    plt.plot(tube2[:,0], tube2[:,1], 'r.')
    plt.axis('equal')
    plt.show()

