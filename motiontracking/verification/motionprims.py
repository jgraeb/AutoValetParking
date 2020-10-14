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
        self.entry_lo = entry_lo
        self.entry_hi = entry_hi
        self.exit_lo = exit_lo
        self.exit_hi = exit_hi
        self.d1 = d1
        self.d2 = d2

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

    def define_parallel_square(self, exit_angle):
        if exit_angle ==0:
            square = Rectangle(1, self.gridsize, self.gridsize, 0, self.gridsize, self.buffer/2, self.gridsize-self.buffer/2)
        elif exit_angle == -45:
            square = Rectangle(1, self.gridsize, self.gridsize, 0, self.gridsize, 0, self.buffer/2)
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

def plot_bounds(alpha_his, w_arr1, alpha_los, w_arr2):
    plt.rcParams.update({"text.usetex": True,"font.family": "sans-serif","font.sans-serif": ["Helvetica"]})
    plt.plot(w_arr1, alpha_his, 'b',  label=r'$\overline{\alpha}$')
    plt.plot(w_arr2, alpha_los, 'r', label=r'$\underline{\alpha}$')
    plt.xlabel('w')
    plt.ylabel('Initial Angle')
    plt.xlim(min(w_arr1[0],w_arr2[0]),max(w_arr1[-1],w_arr2[-1]))
    plt.ylim((-90,90))
    plt.legend()
    plt.title('Angle Bounds for Initial Parallel Square')
    plt.show()

def find_alpha_bounds(avp_spec, square):
    # get parameters
    beta_lo, beta_hi, r, gridsize = avp_spec.extract_params()
    t, d1, d2, w0, w1, y, z = square.extract_params()
    d = d1
    #st()
    # compute bounds for w array
    w_arr = np.linspace(w0,w1,40)
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
        if square.type==2:
            # spec for Upsilon analysis
            spec = Spec(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
        alpha_hi, alpha_lo = find_alphas(UPS, LS)
        alpha_his.append(alpha_hi)
        alpha_los.append(alpha_lo)
    w_arr1, alpha_his = smooth(w_arr,alpha_his)
    w_arr2, alpha_los = smooth(w_arr,alpha_los)
    return alpha_his, w_arr1, alpha_los, w_arr2

def run_testcase(avp_spec): # try verification with first primitive (straight ahead 2 blocks)
    # load the motionprimitives
    channels = get_channels_from_motionprimives()
    prim = channels[0]
    squares = prim.extract_rectangles()
    sols_lo = []
    sols_hi = []
    for idx, square in enumerate(squares):
        alpha_his, w_arr1, alpha_los, w_arr2 = find_alpha_bounds(avp_spec, square)
        sols_lo.append(alpha_los)
        sols_hi.append(alpha_his)
    st()
    plot_bounds(w_arr1, w_arr2, alpha_his, alpha_los)

def run_parallel_square(avp_spec, buffer):
    beta_lo, beta_hi, r, gridsize = avp_spec.extract_params()
    segment = PrimChannel("0", gridsize, buffer)
    square = segment.define_parallel_square(0)
    alpha_his, w_arr1, alpha_los, w_arr2 = find_alpha_bounds(avp_spec, square)
    return alpha_his, w_arr1, alpha_los, w_arr2
    #plot_bounds(w_arr, alpha_his, alpha_los)

def run_paper_example():
    avp_spec = AVPSpec(np.deg2rad(-40), np.deg2rad(10), 45.0, 30)
    square = Rectangle(1, 10, 10, 0, 10, 0, 5)
    alpha_his, w_arr1, alpha_los, w_arr2 = find_alpha_bounds(avp_spec, square)
    #st()
    return alpha_his, w_arr1, alpha_los, w_arr2

if __name__ == '__main__':
    # define exit configuration
    exit_config = [0, 5.0, 1.0] # nomimal_exit_angle, angle_bound, buffer\
    exit_angle = exit_config[0]
    angle_bound = exit_config[1] # in degrees
    buffer = exit_config[2]
    gridsize_pxl = 10 # in pixel (1 pxl = 0.3 m)
    gridsize = gridsize_pxl*SCALE_FACTOR_PLAN # in m
    # AVP car parameters
    r_curv = 6.0
    # create the spec
    avp_spec = AVPSpec(np.deg2rad(exit_angle-angle_bound), np.deg2rad(exit_angle+angle_bound), r_curv, gridsize_pxl)
    # run the motionprimitives testcase
    #run_testcase(avp_spec)
    # sanity check
    #w_arr, alpha_his, alpha_los = run_paper_example()
    # define starting values
    min_diff = 0
    buffer = 0
    alpha_his, w_arr1, alpha_los, w_arr2 = run_parallel_square(avp_spec, buffer)
    plot_bounds(alpha_his, w_arr1, alpha_los, w_arr2)
