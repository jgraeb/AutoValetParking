# Automated Valet Parking - Test of Gazebo simulations
# Tom Andersson
# California Institute of Technology
# June, 2020

import rospy
import time
import sys

from RobotCtrl import RobotCtrl
from ModelCtrl import ModelCtrl
from GazeboCtrl import GazeboCtrl
sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import START_X, START_Y, START_YAW

"""This is a test program for seting and reading linear and rotational velocities
as well as reading position and orientation of a seleccted robot or model in a launched gazebo world with models and robots placed out.
This is done using the RobotCtrl, ModelCtrl and GazeboCtrl class.
The number of robots, pedestrians and obstacles need to match with the variables in global_vars.py"""

rospy.init_node('main')
    
robot_ctrl = RobotCtrl()
model_ctrl = ModelCtrl()
gazebo_ctrl = GazeboCtrl()

robots = [[START_X, START_Y, START_YAW], [2.25, 0.7, 3.14]] 
pedestrians = []#[[2.5, 4.6, 1.57], [2.5, 4.77, 1.57]]
obstacles = []#[[3.63, 2.89, 0], [3.63, 2.73, 0]]
gazebo_ctrl.launch_gazebo(robots, pedestrians, obstacles)

terminate = False

while(not terminate):
    test = int(input("""Select test
                Robot, set linear and rotational velocity: 1
                Robot, get states: 2
                Robot, set velocity and steering angle: 3
                Robot, set acceleration and steering angle: 4
                Robot, set acceletarion and rotational velocity: 5
                Pedestrian, set x velocity: 6
                Pedestrian, set y velocity: 7
                Pedestrian, set x and y velocity: 8
                Pedestrian, set rotational velocity: 9
                Pedestrian, set linear and rotational velocity: 10
                Pedestrian, get states: 11
                Obstacle, get states: 12
                Terminate: 13\n"""))

    if test == 1:
        robot_nbr = int(input("Select robot \n"))
        lin_vel = float(input("Set linear velocity \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        robot_ctrl.set_vel(robot_nbr, lin_vel, rot_vel)

    elif test == 2:
        robot_nbr = int(input("Select robot \n"))
        print(robot_ctrl.get_pos(robot_nbr))
        print(robot_ctrl.get_vel(robot_nbr))

    elif test == 3:
        robot_nbr = int(input("Select robot \n"))
        lin_vel = float(input("Set linear velocity \n"))
        steering_angle = float(input("Set steering angle \n"))
        robot_ctrl.set_vel_n_steer(robot_nbr, lin_vel, steering_angle)

    elif test == 4:
        robot_nbr = int(input("Select robot \n"))
        acc = float(input("Set acceleration \n"))
        steering_angle = float(input("Set steering angle \n"))
        robot_ctrl.set_acc_n_steer(robot_nbr, acc, steering_angle)

    elif test == 5:
        robot_nbr = int(input("Select robot \n"))
        acc = float(input("Set acceleration \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        robot_ctrl.set_acc_n_rot_vel(robot_nbr, acc, rot_vel)

    elif test == 6:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        x_vel = float(input("Set x velocity \n"))
        model_ctrl.set_x_vel(pedestrian_nbr, x_vel)

    
    elif test == 7:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        y_vel = float(input("Set y velocity \n"))
        model_ctrl.set_y_vel(pedestrian_nbr, y_vel)

    elif test == 8:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        x_vel = float(input("Set x velocity \n"))
        y_vel = float(input("Set y velocity \n"))
        model_ctrl.set_lin_vel(pedestrian_nbr, x_vel, y_vel)

    elif test == 9:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        model_ctrl.set_rot_vel(pedestrian_nbr, rot_vel)   

    elif test == 10:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        x_vel = float(input("Set x velocity \n"))
        y_vel = float(input("Set y velocity \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        model_ctrl.set_vel(pedestrian_nbr, x_vel, y_vel, rot_vel)

    elif test == 11:
        pedestrian_nbr = int(input("Select pedestrian \n"))
        print(model_ctrl.get_pedestrian_pos(pedestrian_nbr))
        print(model_ctrl.get_vel(pedestrian_nbr))

    elif test == 12:
        obstacle_nbr = int(input("Select obstacle \n"))
        print(model_ctrl.get_obstacle_pos(obstacle_nbr))

    elif test == 13:
        terminate = True
        gazebo_ctrl.terminate_gazebo()