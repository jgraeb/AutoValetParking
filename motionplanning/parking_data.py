import numpy as np
import _pickle as pickle
from ipdb import set_trace as st
import os
import sys
dirname, filename = os.path.split(os.path.abspath(__file__))
sys.path.append(dirname)

with open(dirname + '/planning_graph_refined.pkl', 'rb') as f:
    planning_graph = pickle.load(f)

parking_spot_list = planning_graph['end_states']
parking_spots = dict()
for idx, spot in enumerate(parking_spot_list):
    parking_spots[idx] = spot
