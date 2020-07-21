# Automated Valet Parking - Test of Gazebo simulations
# Tom Andersson
# California Institute of Technology
# June, 2020

import rospy
import time

from RobotCtrl import RobotCtrl
from ModelCtrl import ModelCtrl
from GazeboCtrl import GazeboCtrl

"""This is a test program for seting and reading linear and rotational velocities
as well as reading position and orientation of a seleccted robot or model in a launched gazebo world with models and robots placed out.
This is done using the RobotCtrl, ModelCtrl and GazeboCtrl class.
The number of robots, pedestrians and obstacles need to match with the variables in global_vars.py"""

rospy.init_node('test_ctrl')
    
robot_ctrl = RobotCtrl()
model_ctrl = ModelCtrl()
gazebo_ctrl = GazeboCtrl()

robots = [[2.75, -1.5, 0], [2.25, -0.7, 3.14]]
pedestrians = [[2.5, -4.6, 1.57], [2.5, -4.77, 1.57]]
obstacles = [[3.63, -2.89, 0], [3.63, -2.73, 0]]
gazebo_ctrl.launch_gazebo(robots, pedestrians, obstacles)

terminate = False

while(not terminate):
    test = int(input("""Select test
                Robot, set linear velocity: 1
                Robot, set rotational velocity: 2
                Robot, set linear and rotational velocity: 3
                Robot, get states: 4
                Model, set x velocity: 5
                Model, set y velocity: 6
                Model, set x and y velocity: 7
                Model, set rotational velocity: 8
                Model, set linear and rotational velocity: 9
                Model, get states: 10
                Terminate: 11\n"""))

    if test == 1:
        robot_nbr = int(input("Select robot \n"))
        lin_vel = float(input("Set linear velocity \n"))
        robot_ctrl.set_lin_vel(robot_nbr, lin_vel)

    elif test == 2:
        robot_nbr = int(input("Select robot \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        robot_ctrl.set_rot_vel(robot_nbr, rot_vel)

    elif test == 3:
        robot_nbr = int(input("Select robot \n"))
        lin_vel = float(input("Set linear velocity \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        robot_ctrl.set_vel(robot_nbr, lin_vel, rot_vel)

    elif test == 4:
        robot_nbr = int(input("Select robot \n"))
        print(robot_ctrl.get_pos(robot_nbr))
        print(robot_ctrl.get_vel(robot_nbr))

    elif test == 5:
        model_name = input("Select model \n")
        x_vel = float(input("Set x velocity \n"))
        model_ctrl.set_x_vel(model_name, x_vel)

    
    elif test == 6:
        model_name = input("Select model \n")
        y_vel = float(input("Set y velocity \n"))
        model_ctrl.set_y_vel(model_name, y_vel)

    elif test == 7:
        model_name = input("Select model \n")
        x_vel = float(input("Set x velocity \n"))
        y_vel = float(input("Set y velocity \n"))
        model_ctrl.set_lin_vel(model_name, x_vel, y_vel)

    elif test == 8:
        model_name = input("Select model \n")
        rot_vel = float(input("Set rotational velocity \n"))
        model_ctrl.set_rot_vel(model_name, rot_vel)   

    elif test == 9:
        model_name = input("Select model \n")
        x_vel = float(input("Set x velocity \n"))
        y_vel = float(input("Set y velocity \n"))
        rot_vel = float(input("Set rotational velocity \n"))
        model_ctrl.set_vel(model_name, x_vel, y_vel, rot_vel)

    elif test == 10:
        model_name = input("Select model \n")
        print(model_ctrl.get_pos(model_name))
        print(model_ctrl.get_vel(model_name))

    elif test == 11:
        terminate = True
        gazebo_ctrl.terminate_gazebo()