# Automated Valet Parking - Test of Robot Control
# Tom Andersson
# California Institute of Technology
# June, 2020

from RobotCtrl import RobotCtrl

"""This is a test program for seting and reading linear and rotational velocities
as well as reading position and orientation of a seleccted turtlebot3.
This is done using the RobotCtrl class"""

robot_ctrl = RobotCtrl()


while(1):
    test = int(input("""Select test
                Set linear velocity: 1
                Set rotational velocity: 2
                Set linear and rotational velocity: 3
                Get states: 4\n"""))

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