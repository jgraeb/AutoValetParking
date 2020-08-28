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
from PIL import ImageFont,ImageDraw  
from numpy import cos, sin, pi
from animation.component import parking_lot
from variables import global_vars
from ipdb  import set_trace as st
from variables.global_vars import SCALE_FACTOR_SIM, SCALE_FACTOR_PLAN
from variables.parking_data import parking_spots_gazebo_test as parking_spots


dir_path = os.path.dirname(os.path.realpath("__file__"))
car_fig = os.path.dirname(dir_path) + '/imglib/blue_car.png'
vehicle_fig = Image.open(car_fig)

gray_car_fig = os.path.dirname(dir_path) + '/imglib/gray_car.png'
gray_vehicle_fig = Image.open(gray_car_fig)

car_scale_factor = 0.2
center_to_axle_dist = 200

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
    theta = -theta
    w, h = square_fig.size
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

def label_spots(ax, background, spots): # put car ids on the assigned parking spots  
    fontpath = 'NotoSans-ExtraBold.ttf'
    try:
        font = ImageFont.truetype(fontpath, 50)
    except:
        fontpath = '../animation/NotoSans-ExtraBold.ttf'
    draw = ImageDraw.Draw(background)
    for spot,car_id in spots.items():
        if spot in range(0,10):
            offset = 160
            offsetB = 10
        elif spot in range(10,19):
            offset = 180
            offsetB = 15
        elif spot in range(19,28):
            offset = 170
            offsetB = -5
        elif spot in range(28,44):
            offset = 175
            offsetB = -5 
        # find x,y location of spot 
        loc = parking_spots[spot]
        x = loc[0]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM
        y = loc[1]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM
        center_x = x + offset*cos(np.deg2rad(-loc[2])) +offsetB
        center_y = y + offset*sin(np.deg2rad(-loc[2]))
        # write car_id
        try:
            n = len(str(car_id.rstrip))
        except:
            n = len(str(car_id))
        draw.ellipse([(center_x-22, center_y-22), (center_x+22, center_y+22)], fill='snow', outline='black')
        if n==1 or int(car_id)<10:
            font = ImageFont.truetype(fontpath, 40)
            draw.text((center_x-10, center_y-27), str(car_id),fill='black',font=font,fontsize=3)
        elif n==2 or int(car_id)>=10:
            font = ImageFont.truetype(fontpath, 35)
            draw.text((center_x-20, center_y-25), str(car_id),fill='black',font=font,fontsize=3)


def draw_car(ax, background,x,y,theta,status,car_id, car=None):
    vehicle_fig = Image.open(car_fig)
    w_orig, h_orig = vehicle_fig.size
    #print(str(w_orig)+','+str(h_orig))
    # convert angle to degrees and positive counter-clockwise
    theta_d = -theta/np.pi * 180
    # set expand=True so as to disable cropping of output image
    vehicle_fig = vehicle_fig.rotate(theta_d, expand = False)
    scaled_vehicle_fig_size  =  tuple([int(car_scale_factor * i) for i in vehicle_fig.size])
    # rescale car 
    vehicle_fig = vehicle_fig.resize(scaled_vehicle_fig_size, Image.ANTIALIAS)
    # at (full scale) the relative coordinates of the center of the rear axle w.r.t. the center of the figure is center_to_axle_dist
    x_corner, y_corner = find_corner_coordinates(-car_scale_factor * center_to_axle_dist, 0, x, y, theta, vehicle_fig)
    background.paste(vehicle_fig, (x_corner, y_corner), vehicle_fig)
    #background.paste(vehicle_fig, (x, y), vehicle_fig)
    if car:
        status = car.status
        if car.parked or car.in_spot:
            status = 'Parked'
        if car.requested:
            if car.status=='Stop' or car.status =='Failure' or car.status == 'Blocked':
                status = car.status
            else:
                status = 'Requested'
        if car.reserved:
            if car.status=='Stop' or car.status =='Failure' or car.status == 'Blocked':
                status = car.status
            else:
                status = 'Reserved'
        ax.text(x_corner,y_corner,str(car.id), color='w', horizontalalignment='center', verticalalignment='center', bbox=dict(facecolor='red', alpha=0.4), fontsize=5)
    
    if status:
        fontpath = 'NotoSans-ExtraBold.ttf'
        try:
            font = ImageFont.truetype(fontpath, 50)
        except:
            fontpath = '../animation/NotoSans-ExtraBold.ttf'
            font = ImageFont.truetype(fontpath, 50)
        draw = ImageDraw.Draw(background) 
        draw.ellipse([(x-25, y-25), (x+25, y+25)], fill='snow', outline='black')
        if status.rstrip() == 'Failure':
            draw.text((x-15, y-35), 'F',fill='red',font=font,fontsize=7)
        elif status.rstrip() == 'Stop':
            draw.text((x-15, y-35), 'S',fill='darkred',font=font,fontsize=7)
        elif status.rstrip() == 'Reserved':
            draw.text((x-15, y-35), 'R',fill='red',font=font,fontsize=7)
        elif status.rstrip() == 'Requested':
            draw.text((x-15, y-35), 'R',fill='darkorange',font=font,fontsize=7)
        elif status.rstrip() == 'Parked':
            draw.text((x-15, y-35), 'P',fill='mediumblue',font=font,fontsize=7)
        elif status.rstrip() == 'Conflict':
            draw.text((x-15, y-35), 'C',fill='darkred',font=font,fontsize=7)
        elif status.rstrip() == 'Driving':
            draw.text((x-15, y-35), 'D',fill='darkgreen',font=font,fontsize=7)
        elif status.rstrip() == 'Idle':
            draw.text((x-10, y-35), 'I',fill='black',font=font,fontsize=7)
        elif status.rstrip() == 'Blocked':
            draw.text((x-15, y-35), 'B',fill='darkred',font=font,fontsize=7)
        elif status.rstrip() == 'Replan':
            draw.text((x-15, y-35), 'R',fill='slateblue',font=font,fontsize=7)
    # put car_ids on cars
    try:
        n = len(str(car_id.rstrip))
    except:
        n = len(str(car_id))
    #if n>=3:
        #car_id = car_id-100
    center_x = x + 90*cos(theta)
    center_y = y + 90*sin(theta)
    draw.ellipse([(center_x-22, center_y-22), (center_x+22, center_y+22)], fill='snow', outline='black')
    if n==1 or int(car_id)<10:
        font = ImageFont.truetype(fontpath, 40)
        draw.text((center_x-10, center_y-27), str(car_id),fill='black',font=font,fontsize=3)
    elif n==2 or int(car_id)>=10:
        font = ImageFont.truetype(fontpath, 35)
        draw.text((center_x-20, center_y-25), str(car_id),fill='black',font=font,fontsize=3)

def show_traj(ax,background, ref):
    for segment in ref:
        ax.plot(segment[:,0]*global_vars.SCALE_FACTOR_SIM*global_vars.SCALE_FACTOR_PLAN, segment[:,1]*global_vars.SCALE_FACTOR_SIM*global_vars.SCALE_FACTOR_PLAN, 'k--')

def draw_obs(ax,background,obs):
    draw = ImageDraw.Draw(background) 
    rad = obs[3]*global_vars.SCALE_FACTOR_SIM*global_vars.SCALE_FACTOR_PLAN
    xcen = obs[0]*global_vars.SCALE_FACTOR_SIM*global_vars.SCALE_FACTOR_PLAN
    ycen = obs[1]*global_vars.SCALE_FACTOR_SIM*global_vars.SCALE_FACTOR_PLAN
    xl = xcen - rad
    yl = ycen - rad
    xr = xcen + rad
    yr = ycen + rad
    draw.ellipse([(xl, yl), (xr, yr)], fill='red', outline='black')

def draw_grey_car(background,x,y,theta):
    gray_vehicle_fig = Image.open(gray_car_fig)
    w_orig, h_orig = gray_vehicle_fig.size
    # convert angle to degrees and positive counter-clockwise
    theta_d = -theta/np.pi * 180
    # set expand=True so as to disable cropping of output image
    gray_vehicle_fig = gray_vehicle_fig.rotate(theta_d, expand = False)
    scaled_vehicle_fig_size  =  tuple([int(car_scale_factor * i) for i in gray_vehicle_fig.size])
    # rescale car 
    gray_vehicle_fig = gray_vehicle_fig.resize(scaled_vehicle_fig_size, Image.ANTIALIAS)
    
    # at (full scale) the relative coordinates of the center of the rear axle w.r.t. the center of the figure is center_to_axle_dist
    x_corner, y_corner = find_corner_coordinates(-car_scale_factor * center_to_axle_dist, 0, x, y, theta, gray_vehicle_fig)
    background.paste(gray_vehicle_fig, (x_corner, y_corner), gray_vehicle_fig)
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