#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Aug 2019

@author: Jiaqi Yan (jiaqi@caltech.edu)
"""

import os
import numpy as np
import random
from PIL import Image
from numpy import cos, sin, pi
from component import parking_lot

dir_path = os.path.dirname(os.path.realpath("__file__"))
car_fig = os.path.dirname(dir_path) + '/imglib/blue_car.png'
vehicle_fig = Image.open(car_fig)

car_scale_factor = 0.24
center_to_axle_dist = 234

cell_coordinates = {'X0': (1412,460,pi),
'X1': (1850,1170,pi/2),
'P1': (1507,1505,-2.1),
'X7': (2165,2450,-pi/2),
'X9': (2719,835,0),
'X11': (2228,300,-pi/2)
}

#'P1': (499,883),
#'P3': (792,883)


def find_corner_coordinates(x_state_center_before, y_state_center_before, x_desired, y_desired, theta, square_fig):
    """
    This function takes an image and an angle then computes the coordinates of the corner.
    If we'd like to put the point specfied by (x_state_center_before, y_state_center_before) at (x_desired, y_desired),
    this function returns the coordinates of the lower left corner of the new image
    """
    w, h = square_fig.size
    theta = -theta
    if abs(w - h) > 1:
        print('Warning: Figure has to be square! Otherwise, clipping or unexpected behavior may occur')
#        warnings.warn("Warning: Figure has to be square! Otherwise, clipping or unexpected behavior may occur")

    R = np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
    x_corner_center_before, y_corner_center_before = -w/2., -h/2. # lower left corner before rotation
    x_corner_center_after, y_corner_center_after = -w/2., -h/2. # doesn't change since figure size remains unchanged

    x_state_center_after, y_state_center_after = R.dot(np.array([[x_state_center_before], [y_state_center_before]])) # relative coordinates after rotation by theta

    x_state_corner_after = x_state_center_after - x_corner_center_after
    y_state_corner_after = y_state_center_after - y_corner_center_after

    # x_corner_unknown + x_state_corner_after = x_desired
    x_corner_unknown = int(x_desired - x_state_center_after + x_corner_center_after)
    # y_corner_unknown + y_state_corner_after = y_desired
    y_corner_unknown = int(y_desired - y_state_center_after + y_corner_center_after)
    return x_corner_unknown, y_corner_unknown

def draw_car(background,x,y,theta):  
    vehicle_fig = Image.open(car_fig)
    w_orig, h_orig = vehicle_fig.size
    # convert angle to degrees and positive counter-clockwise
    theta_d = -theta/np.pi * 180
    # set expand=True so as to disable cropping of output image
    vehicle_fig = vehicle_fig.rotate(theta_d, expand = False)
    scaled_vehicle_fig_size  =  tuple([int(car_scale_factor * i) for i in vehicle_fig.size])
    # rescale car 
    vehicle_fig = vehicle_fig.resize(scaled_vehicle_fig_size, Image.ANTIALIAS)
    
    # at (full scale) the relative coordinates of the center of the rear axle w.r.t. the center of the figure is center_to_axle_dist
    x_corner, y_corner = find_corner_coordinates(car_scale_factor * center_to_axle_dist, 0, x, y, theta, vehicle_fig)
    background.paste(vehicle_fig, (x_corner, y_corner), vehicle_fig)
    #background.paste(vehicle_fig, (x, y), vehicle_fig)
    
def draw_pedestrian(pedestrian,background):
    x, y, theta, current_gait = pedestrian.state
    i = current_gait % pedestrian.film_dim[1]
    j = current_gait // pedestrian.film_dim[1]
    film_fig = Image.open(pedestrian.fig)
    scaled_film_fig_size  =  tuple([int(0.6 * i) for i in film_fig.size])
    film_fig = film_fig.resize( scaled_film_fig_size)
    width, height = film_fig.size
    sub_width = width/pedestrian.film_dim[1]
    sub_height = height/pedestrian.film_dim[0]
    lower = (i*sub_width,j*sub_height)
    upper = ((i+1)*sub_width, (j+1)*sub_height)
    area = (int(lower[0]), int(lower[1]), int(upper[0]), int(upper[1]))
    person_fig = film_fig.crop(area)
    person_fig = person_fig.rotate(180-theta/np.pi * 180 + 90, expand = False)
    x_corner, y_corner = find_corner_coordinates(0., 0, x, y, theta,  person_fig)
    background.paste(person_fig, (int(x_corner), int(y_corner)), person_fig)
    
