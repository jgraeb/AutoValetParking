"""
author: Josefine Graebener
"""


import imageio
import glob
import numpy as np
from PIL import Image
#from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import scipy.misc as smp
import os
import sys
import matplotlib.pyplot as plt
# For voronoi map
#from PathPlanning.VoronoiRoadMap import voronoi_road_map as vrm

# function to skip indices for obstacles
def drange2(start, stop, step):
    numelements = int((stop-start)/float(step))
    for i in range(numelements+1):
            yield start + i*step

# read in obstacles from image
for im_path in glob.glob("imglib/Clean_Layout_solid_no_cars.png"):
     #im = imageio.imread(im_path)
     im = Image.open(im_path)
     pix=im.load()
     print('Size of the image: '+str(im.size))  # Get the width and height of the image for iterating over
     xnum,ynum=im.size
     obs=[]
     obstacleList=[]
     #print(pix[x,y])  
     # Get the RGBA Value of the a pixel of an image
     pix_val = list(im.getdata())
     # make the obstacle map - every pixel which is not white is an obstacle
     R, G, B = im.convert('RGB').split()
     r = R.load()
     g = G.load()
     b = B.load()
     # make a list of black pixels
     # choose every pixel which is at the border to a white pixel
     # do not choose minimum and maximum x value for each y value and 
     # min and max y value for each x value to get rid of outside border
     for i in range(29,xnum-50):
        for j in range(50,ynum-50):
            if(r[i, j] != 255 or g[i, j] != 255 or b[i, j] != 255): # if pixel is black
                if (r[i+1, j] == 255 or g[i+1, j] == 255 or b[i+1, j] == 255): # if neighbor is white
                    obs.append((i, j, 1)) # Make a list of obstacle pixels
                    obstacleList.append((i/100, j/100, 0.1)) # Make a list of obstacle pixels scaled
                if (r[i-1, j] == 255 or g[i-1, j] == 255 or b[i-1, j] == 255): # if neighbor is white
                    obs.append((i, j, 1)) # Make a list of obstacle pixels
                    obstacleList.append((i/100, j/100, 0.1)) # Make a list of obstacle pixels scaled
                if (r[i, j+1] == 255 or g[i, j+1] == 255 or b[i, j+1] == 255): # if neighbor is white
                    obs.append((i, j, 1)) # Make a list of obstacle pixels
                    obstacleList.append((i/100, j/100, 0.1)) # Make a list of obstacle pixels scaled
                if (r[i, j-1] == 255 or g[i, j-1] == 255 or b[i, j-1] == 255): # if neighbor is white
                    obs.append((i, j, 1)) # Make a list of obstacle pixels
                    obstacleList.append((i/100, j/100, 0.1)) # Make a list of obstacle pixels scaled
# now choose every 5th pixel
obs2=[]
obstacleList2=[]
for k in range(len(obs)):
    if (k%5==0): # use every 5th pixel
        obs2.append(obs[k])
        obstacleList2.append(obstacleList[k])
        
# check obs data
data = np.zeros( (xnum,ynum,3), dtype=np.uint8)
# for row in data
for row in obs2:
     for item in row:
          data[row[0],row[1]] = [255,0,0] # makes obstacles red

img = Image.fromarray( data )       # Create a PIL image
#img.show()                      # Show the chosen obstacles as black/red image

#print(obs)
#testobs=obs[0:60]
# to do 
# # generate voronoi grid
# vor = Voronoi(obs)
# voronoi_plot_2d(vor,show_vertices=True, line_colors='orange',line_width=1, line_alpha=0.6, point_size=1)
# plt.show()
#print(size(obs))
try:
    import rrt_star_reeds_shepp as m
except:
    raise

try:
    import rrt_star as rrtstar
except ImportError:
    raise

print('Number of obstacles '+str(len(obs2)))

m.show_animation = False
# get Voronoi map of the obstacles

# Get Path from Voronoi map 
# get waypoints from Voronoi map

# Specify waypoints and connect RRT* through the waypoints by matching location and yaw (and velocity??)
waypoints=[[1844/100,60/100,np.deg2rad(90.0)],[1572/100,463/100,np.deg2rad(180.0)],[1276/100,463/100,np.deg2rad(180.0)],[1434/100,828/100,np.deg2rad(0)],[1891/100,1062/100,np.deg2rad(90.0)],[1891/100,1515/100,np.deg2rad(90.0)],[1512/100,1777/100,np.deg2rad(180.0)],[1032/100,1990/100,np.deg2rad(125.0)],[983/100,1777/100,np.deg2rad(180.0)],[546/100,1777/100,np.deg2rad(180.0)],[245/100,2133/100,np.deg2rad(90.0)],[520/100,2489/100,np.deg2rad(0.0)],[1147/100,2489/100,np.deg2rad(0.0)],[1746/100,2489/100, np.deg2rad(0.0)],[2203/100,2114/100,np.deg2rad(-90.0)],[2203/100,1515/100,np.deg2rad(-90.0)],[2203/100,1062/100,np.deg2rad(-90.0)],[2515/100,835/100,np.deg2rad(0.0)],[2826/100,835/100,np.deg2rad(0.0)],[2627/100,462/100,np.deg2rad(180.0)],[2200/100, 60/100, np.deg2rad(-90.0)]]

# Compute, plot the path and save the x,y, yaw coordinates
iternum=50
# try:
#     pathA = m.mainAVPtest(obstacleList,waypoints[0],waypoints[1],[15.0, 19.0],[0.0, 5.0], max_iter=iternum)
#     plt.plot([x for (x, y,yaw) in pathA], [y for (x, y,yaw) in pathA], '-r')
#     with open('PathA', 'w') as fh:
#         for x, y, yaw in pathA: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathB = m.mainAVPtest(obstacleList,waypoints[1],waypoints[2],[12.0,16.0],[4.0,5.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathB], [y for (x, y, yaw) in pathB], '-r')
#     with open('PathB', 'w') as fh:
#         for x, y, yaw in pathB: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathC = m.mainAVPtest(obstacleList,waypoints[2],waypoints[3],[9.5,15.0],[4.0,9.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathC], [y for (x, y, yaw) in pathC], '-r')
#     with open('PathC', 'w') as fh:
#         for x, y, yaw in pathC: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# # try:
#     pathD = m.mainAVPtest(obstacleList,waypoints[3],waypoints[4],[14.0,19.0],[7.5, 11.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathD], [y for (x, y, yaw) in pathD], '-r')
#     with open('PathD', 'w') as fh:
#         for x, y, yaw in pathD: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathE = m.mainAVPtest(obstacleList,waypoints[4],waypoints[5],[18.0,19.5],[10.0,15.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathE], [y for (x, y, yaw) in pathE], '-r')
#     with open('PathE', 'w') as fh:
#         for x, y, yaw in pathE: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathF = m.mainAVPtest(obstacleList,waypoints[5],waypoints[6],[15.0,19.5],[15.0,18.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathF], [y for (x, y, yaw) in pathF], '-r')
#     with open('PathF', 'w') as fh:
#         for x, y, yaw in pathF: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathG = m.mainAVPtest(obstacleList,waypoints[6],waypoints[7],[10.0,15.5],[17.0, 20.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathG], [y for (x, y, yaw) in pathG], '-r')
#     with open('PathG', 'w') as fh:
#         for x, y, yaw in pathG: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass
# try:
#     pathH = m.mainAVPtest(obstacleList,waypoints[7],waypoints[8],[9.5,15.0],[17.0, 20.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathH], [y for (x, y, yaw) in pathH], '-r')
#     with open('PathH', 'w') as fh:
#         for x, y, yaw in pathH: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathI = m.mainAVPtest(obstacleList,waypoints[8],waypoints[9],[5.0,10.0],[17.0, 18.2],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathI], [y for (x, y, yaw) in pathI], '-r')
#     with open('PathI', 'w') as fh:
#         for x, y, yaw in pathI: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathJ = m.mainAVPtest(obstacleList,waypoints[9],waypoints[10],[2.0,6.0],[17.0, 22.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathJ], [y for (x, y, yaw) in pathJ], '-r')
#     with open('PathJ', 'w') as fh:
#         for x, y, yaw in pathJ: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathK = m.mainAVPtest(obstacleList,waypoints[10],waypoints[11],[2.0,5.5],[21.0, 25.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathK], [y for (x, y, yaw) in pathK], '-r')
#     with open('PathK', 'w') as fh:
#         for x, y, yaw in pathK: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathL = m.mainAVPtest(obstacleList,waypoints[11],waypoints[12],[5.0,12.0],[24.0, 25.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathL], [y for (x, y, yaw) in pathL], '-r')
#     with open('PathL', 'w') as fh:
#         for x, y, yaw in pathL: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathM = m.mainAVPtest(obstacleList,waypoints[12],waypoints[13],[11.0,18.0],[24.0, 25.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathM], [y for (x, y, yaw) in pathM], '-r')
#     with open('PathM', 'w') as fh:
#         for x, y, yaw in pathM: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathN = m.mainAVPtest(obstacleList,waypoints[13],waypoints[14],[17.0,22.5],[20.5, 25.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathN], [y for (x, y, yaw) in pathN], '-r')
#     with open('PathN', 'w') as fh:
#         for x, y, yaw in pathN: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathO = m.mainAVPtest(obstacleList,waypoints[14],waypoints[15],[21.5,22.5],[21.0, 15.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathO], [y for (x, y, yaw) in pathO], '-r')
#     with open('PathO', 'w') as fh:
#         for x, y, yaw in pathO: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathP = m.mainAVPtest(obstacleList,waypoints[15],waypoints[16],[21.5,22.5],[15.5, 10.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathP], [y for (x, y, yaw) in pathP], '-r')
#     with open('PathP', 'w') as fh:
#         for x, y, yaw in pathP: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathQ = m.mainAVPtest(obstacleList,waypoints[16],waypoints[17],[21.0,25.5],[10.5, 7.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathQ], [y for (x, y, yaw) in pathQ], '-r')
#     with open('PathQ', 'w') as fh:
#         for x, y, yaw in pathQ: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathR = m.mainAVPtest(obstacleList,waypoints[17],waypoints[18],[25.0,28.5],[8.0, 9.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathR], [y for (x, y, yaw) in pathR], '-r')
#     with open('PathR', 'w') as fh:
#         for x, y, yaw in pathR:
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathS = m.mainAVPtest(obstacleList,waypoints[18],waypoints[19],[25.0,35.0],[4.0, 8.5],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathS], [y for (x, y, yaw) in pathS], '-r')
#     with open('PathS', 'w') as fh:
#         for x, y, yaw in pathS: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# try:
#     pathT = m.mainAVPtest(obstacleList,waypoints[19],waypoints[20],[21.0,27.5],[5.5, 0.0],max_iter=iternum)
#     plt.plot([x for (x, y, yaw) in pathT], [y for (x, y, yaw) in pathT], '-r')
#     with open('PathT', 'w') as fh:
#         for x, y, yaw in pathT: 
#             fh.write('{} {} {}\n'.format(x, y, yaw))
# except:
#     pass

# Plot the stored paths
dataA = np.flipud(np.genfromtxt("nominal_paths/pathA",delimiter=" "))
dataB = np.flipud(np.genfromtxt("nominal_paths/pathB",delimiter=" "))
dataC = np.flipud(np.genfromtxt("nominal_paths/pathC",delimiter=" "))
dataD = np.flipud(np.genfromtxt("nominal_paths/pathD",delimiter=" "))
dataE = np.flipud(np.genfromtxt("nominal_paths/pathE",delimiter=" "))
dataF = np.flipud(np.genfromtxt("nominal_paths/pathF",delimiter=" "))
dataG = np.flipud(np.genfromtxt("nominal_paths/pathG",delimiter=" "))
dataH = np.flipud(np.genfromtxt("nominal_paths/pathH",delimiter=" "))
dataI = np.flipud(np.genfromtxt("nominal_paths/pathI",delimiter=" "))
dataJ = np.flipud(np.genfromtxt("nominal_paths/pathJ",delimiter=" "))
dataK = np.flipud(np.genfromtxt("nominal_paths/pathK",delimiter=" "))
dataL = np.flipud(np.genfromtxt("nominal_paths/pathL",delimiter=" "))
dataM = np.flipud(np.genfromtxt("nominal_paths/pathM",delimiter=" "))
dataN = np.flipud(np.genfromtxt("nominal_paths/pathN",delimiter=" "))
dataO = np.flipud(np.genfromtxt("nominal_paths/pathO",delimiter=" "))
dataP = np.flipud(np.genfromtxt("nominal_paths/pathP",delimiter=" "))
dataQ = np.flipud(np.genfromtxt("nominal_paths/pathQ",delimiter=" "))
dataR = np.flipud(np.genfromtxt("nominal_paths/pathR",delimiter=" "))
dataS = np.flipud(np.genfromtxt("nominal_paths/pathS",delimiter=" "))
dataT = np.flipud(np.genfromtxt("nominal_paths/pathT",delimiter=" "))

data=np.vstack((dataA,dataB,dataC,dataD,dataE,dataF,dataG,dataH,dataI,dataJ,dataK,dataL,dataM,dataN,dataO,dataP,dataQ,dataR,dataS,dataT))
plt.plot([x for (x, y, yaw) in data], [y for (x, y, yaw) in data], '-r')
with open('pathdata', 'w') as fh:
    for x, y, yaw in data:
        fh.write('{} {} {}\n'.format(x, y, yaw))


# plot specified waypoints
for (ox, oy, oyaw) in waypoints:    
     plt.plot(ox, oy, "xr")

# plot obstacles
for (ox, oy, size) in obstacleList2:    
     plt.plot(ox, oy, "ok", ms=30 * size)
plt.show()



