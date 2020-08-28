# Automated Valet Parking - Test of Gazebo simulations, trajectory control
# Tom Andersson
# California Institute of Technology
# August, 2020


"""Used for testing the trajectory control of a turtlebot using state feedback with feedforward.
    Does not work due to the system being noncontrollable at certain states. (eg when v=0)"""
    
import rospy
import math
import sys
import numpy as np
import control

from RobotCtrl import RobotCtrl
from GazeboCtrl import GazeboCtrl

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import back_to_sim_front_wheel_length as length

rospy.init_node('test_trajectory_ctrl')
    
robot_ctrl = RobotCtrl()

gazebo_ctrl = GazeboCtrl()

x = 2.75
y = -1.5
yaw = 0

gazebo_ctrl.launch_gazebo([x, y, yaw], [], [])
rospy.sleep(2)

state = np.transpose([x, y, yaw])
state_m = state

h = 0.01
rospy_rate = rospy.Rate(1/h)
u = np.transpose([0.01, 0])
r = np.transpose([2.75, -1.5, 0])

yaw = 0.1
Phi = np.array([[1, 0, -u[0]*math.sin(yaw)*h],
        [0, 1, u[0]*math.cos(yaw)*h],
        [0, 0, 1]])
print(Phi)
Gamma = np.array([[math.cos(yaw)*h-math.sin(yaw)*math.tan(u[1])*h**2*u[0]/(2*length), -math.sin(yaw)*h**2*u[0]**2/(2*length*math.cos(u[1])**2)],
        [math.sin(yaw)*h+math.cos(yaw)*math.tan(u[1])*h**2*u[0]/(2*length), math.cos(yaw)*h**2*u[0]**2/(2*length*math.cos(u[1])**2)],
        [math.tan(u[1])*h/length, h*u[0]/(length*math.cos(u[1])**2)]])
print(Gamma)
I = np.array([[1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])

C = I

# 1 pole in wc and 2 complex conjugated poles with relative damping = 5 and natural frequancy = wc, poles for model twice as fast
wc = 0.015
poles_c = wc*np.roots([1, 2, 2, 1])
poles_m_c = 2*wc*np.roots([1, 2, 2, 1])
#poles_c = np.array([-10, -10, -20])
#poles_m_c = 2*poles_c
poles = np.exp(poles_c*h)
poles_m = np.exp(poles_m_c*h)
print(poles)
print(poles_m)

L = np.array(control.place(Phi, Gamma, poles))
print(L)
Lm = np.array(control.place(Phi, Gamma, poles_m))
Lr = np.transpose(np.divide(1, np.matmul(np.matmul(C, np.linalg.inv(I-Phi+np.matmul(Gamma,L))), Gamma)))
print(Lr)

while not rospy.is_shutdown():
    y = robot_ctrl.get_pos(0)
    u_ff = -np.matmul(Lm, state_m) + np.matmul(Lr, r)
    u = np.matmul(L, (state_m-state)) + u_ff
    print(u)
    robot_ctrl.set_vel_n_steer(0, u[0], u[1])
    
    Phi = np.array([[1, 0, -u[0]*math.sin(state[2])*h],
        [0, 1, u[0]*math.cos(state[2])*h],
        [0, 0, 1]])

    Gamma = np.array([[math.cos(state[2])*h-math.sin(state[2])*math.tan(u[1])*h**2*u[0]/(2*length), -math.sin(state[2])*h**2*u[0]**2/(2*length*math.cos(u[1])**2)],
        [math.sin(state[2])*h+math.cos(state[2])*math.tan(u[1])*h**2*u[0]/(2*length), math.cos(state[2])*h**2*u[0]**2/(2*length*math.cos(u[1])**2)],
        [math.tan(u[1])*h/length, h*u[0]/(length*math.cos(u[1])**2)]])

    L = control.place(Phi, Gamma, poles)
    Lm = control.place(Phi, Gamma, poles_m)
    Lr = np.divide(1, np.matmul(np.matmul(C, np.linalg.inv(I-Phi+np.matmul(Gamma,L))), Gamma))

    state_m = np.matmul(Phi, state_m) + np.matmul(Gamma, u_ff)
    state = robot_ctrl.get_pos(0)

    rospy_rate.sleep()