#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Nov 2019

@author: Jiaqi Yan 

"""

import sys
import os
sys.path.append('..') # enable importing modules from an upper directory:
from prepare.helper import *
from tulip_spec.simplestspec_ctrl import ExampleCtrl
import prepare.user as user
import component.pedestrian as pedestrian
from PIL import Image

path =[]
dir_path = os.path.dirname(os.path.realpath(__file__))
#intersection_fig = dir_path + "/components/imglib/intersection_states/intersection_lights.png"
pathfile = os.path.dirname(dir_path) + '/motionplanning/nominal_paths/pathdata_read.txt'
with open(pathfile,'r') as f:
	for line in f:
		path.append(list(line.strip('\n').split(',')))

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
    
# set to True to save video
save_video = False

# creates figure
fig = plt.figure()
ax = fig.add_axes([0,0,1,1]) # get rid of white border
plt.axis('off')

background = parking_lot.get_background()

start_walk_lane = (2908,665)
end_walk_lane = (3160,665)
# add one pedestrian to the scene
pedestrian = pedestrian.Pedestrian(pedestrian_type='1')
pedestrian.prim_queue.enqueue(((start_walk_lane, end_walk_lane, 60), 0))

dt = 0.1
path_idx = 0
def animate(frame_idx): # update animation by dt
    global background, path_idx, current_loc
    ax.clear()
    current_loc = path[path_idx][0].split()
    path_idx += 1
    x = float(current_loc[0])*360
    y = float(current_loc[1])*380
    print(y)
    theta = float(current_loc[2])
    #x,y,theta = [5.0*367, 0.2*367+145, 1.5707963267948966]
    draw_car(background,x,y,theta) # draw cars to background
    
    if pedestrian.state[0] < end_walk_lane[0]: # if not at the destination
        pedestrian.prim_next(dt)
        draw_pedestrian(pedestrian,background) 
        
    # update background
    the_parking_lot = [ax.imshow(background)] # update the stage
    background.close()
    background = parking_lot.get_background()
    #print(frame_idx)
    all_artists = the_parking_lot 
    return all_artists
t0 = time.time()
animate(0)
t1 = time.time()
interval = (t1 - t0)
ani = animation.FuncAnimation(fig, animate, frames=200, interval=10**3, blit=True, repeat=False) # by default the animation function loops so set repeat to False in order to limit the number of frames generated to num_frames
if save_video:
    #Writer = animation.writers['ffmpeg']
    writer = animation.FFMpegWriter(fps = 1, metadata=dict(artist='Easy Park Simulator'), bitrate=None)
    now = str(datetime.datetime.now())
    ani.save('../movies/' + now + '.mp4', dpi=200, writer=writer)
plt.show()
t2 = time.time()
print('Total elapsed time: ' + str(t2-t0))


