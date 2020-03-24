#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Mar 2020

@author: Jiaqi Yan 

"""
import sys
sys.path.append('..') # enable importing modules from an upper directory:
from PIL import Image
from prepare.helper import *
import time, platform, warnings, matplotlib, random
import datetime
if platform.system() == 'Darwin': # if the operating system is MacOS
#    matplotlib.use('macosx')
    matplotlib.use('Qt5Agg')
else: # if the operating system is Linux or Windows
    try:
        import PySide2 # if pyside2 is installed
        matplotlib.use('Qt5Agg')
    except ImportError:
        warnings.warn('Using the TkAgg backend, this may affect performance. Consider installing pyside2 for Qt5Agg backend')
        matplotlib.use('TkAgg') # this may be slower
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from component import parking_lot
import component.pedestrian as Pedestrian
    
# set to True to save video
save_video = False
SCALE_FACTOR = 35 # scale from meters to pixels in simulation

# creates figure
fig = plt.figure()
ax = fig.add_axes([0,0,1,1]) # get rid of white border
plt.axis('off')

background = parking_lot.get_background()

start_walk_lane = (2908,665)
end_walk_lane = (3160,665)


dt = 0.1

def animate(frame_idx): # update animation by dt
    global background
    ax.clear()
    car_at_this_frame = []
    # scale to the large topo
    xscale = 10.5
    yscale = 10.3
    xoffset = 0
    yoffset = 40
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
             draw_car(background, float(car[0])*SCALE_FACTOR,float(car[1])*SCALE_FACTOR+yoffset,float(car[2]))
        f.close() 
                
    # update background
    the_parking_lot = [ax.imshow(background)] # update the stage
    background.close()
    background = parking_lot.get_background()
    all_artists = the_parking_lot
    return all_artists

ani = animation.FuncAnimation(fig, animate, frames=30, interval=10**3, blit=True, repeat=False) # by default the animation function loops so set repeat to False in order to limit the number of frames generated to num_frames
if save_video:
    #Writer = animation.writers['ffmpeg']
    writer = animation.FFMpegWriter(fps = 1, metadata=dict(artist='Auto Park Simulator'), bitrate=None)
    now = str(datetime.datetime.now())
    ani.save('../movies/' + now + '.mp4', dpi=200, writer=writer)
plt.show()
t2 = time.time()
print('Total elapsed time: ' + str(t2-t0))
