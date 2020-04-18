#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Mar 2020

@author: Jiaqi Yan 

"""
import sys
import pickle
sys.path.append('..') # enable importing modules from an upper directory:
from PIL import Image
from prepare.helper import *
import time, platform, warnings, matplotlib, random
import datetime
# if platform.system() == 'Darwin': # if the operating system is MacOS
# #    matplotlib.use('macosx')
#     matplotlib.use('Qt5Agg')
# else: # if the operating system is Linux or Windows
#     try:
#         import PySide2 # if pyside2 is installed
#         matplotlib.use('Qt5Agg')
#     except ImportError:
#         warnings.warn('Using the TkAgg backend, this may affect performance. Consider installing pyside2 for Qt5Agg backend')
#         matplotlib.use('TkAgg') # this may be slower
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from component import parking_lot
from variables.global_vars import *
from variables.parking_data import parking_spots, grey_cars
import components
import component.pedestrian as Pedestrian
    
add_parked_cars = True
# set to True to save video
save_video = False
# creates figure
fig = plt.figure()
ax = fig.add_axes([0,0,1,1]) # get rid of white border
plt.axis('off')
frame = plt.gca()
frame.axes.get_yaxis().set_visible(False)
frame.axes.get_xaxis().set_visible(False)

background = parking_lot.get_background()

start_walk_lane = (2908,665)
end_walk_lane = (3160,665)


dt = 0.1

def animate(frame_idx): # update animation by dt
    global background
    ax.clear()
    car_at_this_frame = []
    # scale to the large topo
    xoffset = 0
    yoffset = 0
    dt = 0.1

    with open('car_pos.txt') as f:
        i = 0
        begin = 0
        end = 0
        lines = f.readlines() 
        for line in lines: 
            i += 1
            if line.strip() == 'FRAME'+ str(frame_idx):
                begin = i
            if line.strip() == 'FRAME'+ str(frame_idx+1):
                end = i
                break
       
        car_at_this_frame = lines[begin:end-1]
        for car in car_at_this_frame:
             car = car.split(' ')     
             try:
                 draw_car(background, float(car[0])*SCALE_FACTOR_SIM,float(car[1])*SCALE_FACTOR_SIM+yoffset,float(car[2]))
             except EOFError:
                 break    
        f.close() 
        
    with open('pedestrian_file.pkl','rb') as f:
        i = 0
        begin = 0
        end = 0
        while True:
            i += 1
            pedestrian=pickle.load(f)
            if pedestrian == 'FRAME'+ str(frame_idx)+'\n':
                begin = i
            if pedestrian == 'FRAME'+ str(frame_idx+1)+'\n':
                end = i
                break
    
    with open('pedestrian_file.pkl','rb') as f:
        i = 0
        while True:
            i = i+1
            try:
                pedestrian=pickle.load(f)
                if i > begin and i < end:
                    draw_pedestrian(pedestrian,background)
                if i >= end:
                    break
            except EOFError:
                break
    
    # update background
    if add_parked_cars:
        for key,value in grey_cars.items():
            xd = int(value[0]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+xoffset)
            yd = int(value[1]*SCALE_FACTOR_PLAN*SCALE_FACTOR_SIM+yoffset)
            hd = np.deg2rad(-value[2])
            draw_grey_car(background,float(xd),float(yd),float(hd))
    the_parking_lot = [ax.imshow(background)] # update the stage
    background.close()
    background = parking_lot.get_background()
    all_artists = the_parking_lot
    return all_artists

ani = animation.FuncAnimation(fig, animate, frames=3000, interval=10**3, blit=True, repeat=False) # by default the animation function loops so set repeat to False in order to limit the number of frames generated to num_frames
if save_video:
    #Writer = animation.writers['ffmpeg']
    writer = animation.FFMpegWriter(fps = 10, metadata=dict(artist='Auto Park Simulator'), bitrate=None)
    now = str(datetime.datetime.now())
    ani.save('../movies/' + now + '.mp4', dpi=200, writer=writer)
plt.show()
#t2 = time.time()
#print('Total elapsed time: ' + str(t2-t0))
