# A-star algorithm and image processing
# Tung M. Phan
# California Institute of Technology
# February 10th, 2020

import planning_graph
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
np_bitmap = csv_bitmap_to_numpy_bitmap('bitmap')
anchor = (1, 5)
sampled = uniform_sample_grid_points(np_bitmap, anchor = [32, 22], grid_size = 10)
plt.plot(sampled[:,0], sampled[:,1], '.')
plt.show()

def to_planning_graph():
    pass

def make_grid(m, n):
    for i in range(m):
        for j in range(n):
            print(i,j)


def A_star(start, end, weighted_graph):
    def get_manhattan_distance(start, end):
        return np.abs(start.x - end.x) + np.abs(start.y - end.y)

    checked = []
    unchecked = []
