# Helper functions for Game Component
# Josefine Graebener
# California Institute of Technology
# June 2020

import numpy as np
from variables.global_vars import SCALE_FACTOR_PLAN
import sys
sys.path.append('/anaconda3/lib/python3.7/site-packages')
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity
from shapely.ops import nearest_points, split
from variables.parking_data import parking_spots
from ipdb import set_trace as st


def rad2rad(angle): # convert multiples of pi to intervat [-pi,pi)
    if abs(angle)>np.pi:
        if angle > np.pi:
            angle = np.mod(angle, 2*np.pi)
            diff = angle - np.pi
            angle = diff - np.pi
        elif angle < -np.pi:
            angle = np.mod(angle, 2*np.pi)
            diff = angle + np.pi
            angle = np.pi - diff
    return angle

def remove_duplicates(ref): # remove duplicates in path
    del_idx = []
    for i in range(0,len(ref)-1):
        if np.all(ref[i]==ref[i+1]):
            del_idx.append(i)
    del_idx = np.flip(del_idx)
    for k in del_idx:
        ref.pop(k)
    return ref

def make_line(ref): # function to give shapely line from path and convert to meters
    ref = np.concatenate(ref, axis=0) 
    ref = ref.tolist()
    ref = remove_duplicates(ref)
    ref = [(entry[0]*SCALE_FACTOR_PLAN,entry[1]*SCALE_FACTOR_PLAN) for entry in ref]
    ref = LineString(ref)
    return ref