# Josefine Graebener, Tung Phan
# California Institute of Technology
# Definition of the rectangular channels for the motion primitives
# to be used in the algorithms in Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots by R. V. Cowlagi and P. Tsiotras
# https://users.wpi.edu/~rvcowlagi/publications/2014-tro-cbta.pdf
# and https://users.wpi.edu/~rvcowlagi/publications/rvcowlagi-phdthesis.pdf

import numpy as np
from typing import List, Tuple
from CBTA_algs import *
import sys
sys.path.append('../..') # enable importing modules from an upper directory:
from variables.global_vars import SCALE_FACTOR_PLAN
from ipdb import set_trace as st
import json
import matplotlib.pyplot as plt


class AVPSpec:
    def __init__(self, beta_lo: float, beta_hi: float, rad : float, gridsize : int):
        """
        Init class object with params.
        :param beta_lo: Lower angle bound at the exit
        :param beta_hi: Upper angle bound at the exit
        :param alpha_lo: Lower angle bound at the entrance
        :param alpha_hi: Upper angle bound at the entrance
        :param rad: Car Radius of curvature
        :param gridsize: Size of the planning grid in pixels
        """
        self.beta_lo = beta_lo
        self.beta_hi = beta_hi
        # self.alpha_lo = alpha_lo
        # self.alpha_hi = alpha_hi
        self.rad = rad
        self.gridsize = gridsize

    def extract_params(self) -> Tuple[float]:
        """
        Return all parameters as a tuple.
        """
        return self.beta_lo, self.beta_hi, self.rad, self.gridsize

class Rectangle:
    def __init__(self, type: int, d1: float, d2:float, entry_lo: float, entry_hi: float, exit_lo: float, exit_hi: float):
        self.type = type
        self.d1 = d1
        self.d2 = d2
        self.entry_lo = entry_lo
        self.entry_hi = entry_hi
        self.exit_lo = exit_lo
        self.exit_hi = exit_hi


    def extract_params(self):
        """
        Return all parameters as a tuple.
        """
        return self.type, self.d1, self.d2, self.entry_lo, self.entry_hi, self.exit_lo, self.exit_hi


class PrimChannel:
    def __init__(self, number:int, gridsize_pxl : float, buffer: float):
        """
        Init class object with params.
        :param gridsize_pxl: Grid size in pixels
        :param tubesize: Tube size for motion primitive
        :param buffer: Additional buffer for uncertainty
        :num_segments: Number of segments
        """
        self.number = number
        self.gridsize_pxl = gridsize_pxl
        self.gridsize = self.gridsize_pxl*SCALE_FACTOR_PLAN
        #self.tubesize = tubesize
        self.buffer = buffer
        #self.num_segments = num_segments
        self.coords = []
        self.rectangles = []

    def set_coordinates(self,nodelist):
        self.coords = [[item[0]*self.gridsize,item[1]*self.gridsize] for item in nodelist]
        self.create_rectangles()

    def create_rectangles(self): # automate this
        for k in range(0,len(self.coords)):
            rectangle = Rectangle(1, self.gridsize, self.gridsize, self.buffer/2, self.gridsize-self.buffer/2, self.buffer/2, self.gridsize-self.buffer/2)
            self.rectangles.append(rectangle)

    def define_parallel_square(self):
        square = Rectangle(1, self.gridsize, self.gridsize, 0, self.gridsize, self.buffer/2, self.gridsize-self.buffer/2)
        #st()
        return square

    def define_adjacent_square(self):
        square = Rectangle(2, self.gridsize, self.gridsize, self.buffer/2, self.gridsize-self.buffer/2, self.buffer/2, self.gridsize-self.buffer/2)
        return square

    def extract_rectangles(self):
        return self.rectangles

    def extract_params(self) -> Tuple[float]:
        """
        Return all parameters as a tuple.
        """
        return self.coords, self.gridsize_pxl, self.buffer

def get_channels_from_motionprimives():
    # load the motionprimitives
    f = open('../../motionplanning/10px_prims.json',)
    primdata = json.load(f)
    # create the rectangular channels
    channels = []
    for i in range(0,2):#len(primdata)):
        # create channels from the motionprimitives
        channel = PrimChannel(i, 10, 1)
        channel.set_coordinates(primdata[str(i)]["node_sequence"])
        channels.append(channel)
    return channels

def plot_bounds(w_arr, alpha_his, alpha_los):
    plt.plot(w_arr, alpha_his, 'b',  label='Alpha High')
    plt.plot(w_arr, alpha_los, 'r', label='Alpha Low')
    plt.xlabel('w')
    plt.ylabel('alphas')
    plt.ylim((-90,90))
    plt.legend()
    plt.title('Upper and lower Angle Bounds')
    plt.show()

def find_alpha_bounds(avp_spec, square):
    # get parameters
    beta_lo, beta_hi, r, gridsize = avp_spec.extract_params()
    t, d1, d2, w0, w1, y, z = square.extract_params()
    d = d1
    #st()
    # compute bounds for w array
    w_arr = np.linspace(w0,w1,100)
    alpha_his = []
    alpha_los = []
    #st()
    for w_cur in w_arr:
        if square.type==1:
            # spec for Upsilon analysis
            spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
            #st()
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
            #st()
            # alter spec for Lambda analysis (Flip horizontally)
            spec = ChannelProblemSpecifications(d=d,y=d-z,z=d,w=d-w_cur,r=r,beta_lo=-beta_hi,beta_hi=-beta_lo)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            alpha_hi, alpha_lo = find_alphas(UPS, LS)
        if square.type==2:
            # spec for Upsilon analysis
            spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
            alpha_hi, alpha_lo = find_alphas(UPS, LS)
        alpha_his.append(alpha_hi)
        alpha_los.append(alpha_lo)
    return alpha_his, alpha_los, w_arr

def run_testcase(avp_spec): # try verification with first primitive (straight ahead 2 blocks)
    # load the motionprimitives
    channels = get_channels_from_motionprimives()
    prim = channels[0]
    squares = prim.extract_rectangles()
    sols_lo = []
    sols_hi = []
    for idx, square in enumerate(squares):
        alpha_his, alpha_los, w_arr = find_alpha_bounds(avp_spec, square)
        sols_lo.append(alpha_los)
        sols_hi.append(alpha_his)
    st()
    plot_bounds(w_arr, alpha_his, alpha_los)

def run_adjacent_square(avp_spec, buffer):
    beta_lo, beta_hi, r, gridsize = avp_spec.extract_params()
    segment = PrimChannel("0", gridsize, buffer)
    square = segment.define_adjacent_square()
    alpha_his, alpha_los, w_arr = find_alpha_bounds(avp_spec, square)
    angle_bound_entrance = find_value_at_buffer(w_arr, buffer, alpha_los)
    buffer_entrance = find_buffer_at_value(w_arr, -27.5, alpha_los)
    angle_bound_exit = np.rad2deg(beta_hi)
    tubesize = gridsize*SCALE_FACTOR_PLAN - buffer
    primdata = {'Adjacent':[{'gridsize' : gridsize, 'angle_bound_entrance': np.abs(angle_bound_entrance) ,'angle_bound_exit': angle_bound_exit, 'buffer': buffer, 'tubesize': tubesize}]}
    st()
    return w_arr, alpha_his, alpha_los, primdata

def run_parallel_square(avp_spec, buffer):
    beta_lo, beta_hi, r, gridsize = avp_spec.extract_params()
    segment = PrimChannel("0", gridsize, buffer)
    square = segment.define_parallel_square()
    alpha_his, alpha_los, w_arr = find_alpha_bounds(avp_spec, square)
    angle_bound_entrance = find_value_at_buffer(w_arr, buffer, alpha_los)
    buffer_entrance = find_buffer_at_value(w_arr, -27.5, alpha_los)
    angle_bound_exit = np.rad2deg(beta_hi)
    tubesize = gridsize*SCALE_FACTOR_PLAN - buffer
    primdata = {'Parallel':[{'gridsize' : gridsize, 'angle_bound_entrance': np.abs(angle_bound_entrance) ,'angle_bound_exit': angle_bound_exit, 'buffer': buffer, 'tubesize': tubesize}]}
    return w_arr, alpha_his, alpha_los, primdata

def find_buffer_at_value(w_arr, val, alpha_los):
    idx = (np.abs(np.array(alpha_los) - val)).argmin()
    return w_arr[idx]

def find_value_at_buffer(w_arr, buffer, alpha_los):
    idx = (np.abs(w_arr - buffer/2)).argmin()
    #idx2 = (np.abs(array - (gridsize*SCALE_FACTOR_PLAN-buffer/2))).argmin()
    #a = alpha_his[idx2]
    return alpha_los[idx]

def run_paper_example():
    avp_spec = AVPSpec(np.deg2rad(-40), np.deg2rad(10), 45.0, 30)
    square = Rectangle(1, 10, 10, 0, 10, 0, 5)
    alpha_his, alpha_los, w_arr = find_alpha_bounds(avp_spec, square)
    return w_arr, alpha_his, alpha_los

def export_json_file(primdata):
    with open('primdata.json') as f:
        dict = json.load(f)
    dict.update(primdata)
    data = json.dumps(dict, indent=2)
    resultfile = 'primdata.json'
    with open(resultfile, 'w') as f:
        f.write(data)

def compute_buffer_tubesize(w_arr, alpha_his, alpha_los):
    for idx,alpha_hi in enumerate(alpha_his):
        if np.abs(alpha_hi) < angle_bound:
            print(str(idx)+str(alpha_hi))
            index1 = idx
            break
    for idx,alpha_lo in enumerate(alpha_los):
        if np.abs(alpha_lo) > angle_bound:
            print(str(idx)+str(alpha_lo))
            index2 = idx
            break
    a = w_arr[index1]
    b = w_arr[index2]
    tubesize = a - b
    buffer = gridsize - tubesize
    return buffer, tubesize

if __name__ == '__main__':
    # define exit configuration
    exit_config = [27.5, 1.15151516] # angle_bound, buffer
    angle_bound = exit_config[0] # in degrees
    buffer = exit_config[1]
    gridsize_pxl = 10 # in pixel
    gridsize = gridsize_pxl*SCALE_FACTOR_PLAN # in m
    # AVP car parameters
    r_curv = 5.0
    # create the spec
    avp_spec = AVPSpec(np.deg2rad(-angle_bound), np.deg2rad(angle_bound), r_curv, gridsize_pxl)
    # run the motionprimitives testcase
    w_arr, alpha_his, alpha_los, primdata = run_parallel_square(avp_spec, buffer)
    export_json_file(primdata)
    plot_bounds(w_arr, alpha_his, alpha_los)
