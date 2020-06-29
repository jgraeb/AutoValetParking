# Automated Valet Parking


![](movies/replanning.gif)

An example of modular contract-based design using a directive/response architecture.

## Specific for the gazebo_ros branch:
This branch introduces gazebo simulations of turtlebot3 for the automated valet parking. The functionallity and this README are currently under construction.
To run this branch you need to have the AutomatedValetParking package in a catkin workspace for ROS melodic.

### How to run a test for setting velocities to turtlebots in an empty gazebo world:
1. Open a new terminal and go into your catkin workspace and source the generated setup file with the command: **source devel/setup.bash**
2. Launch the turtlebots in an empty world with the command: **roslaunch gazebo main.launch**
3. Open an new terminal and go to this folder AutoValetParking/gazebo/src. To begin publishing velocity topics to the turtlebots run the program robot_commander.py using the command: **python3 robot_commander.py**.
4. Open an new terminal and go to this folder again AutoValetParking/gazebo/src. To set new velocities to the turtlebots run the program test_publisher.py using the command: **python3 test_publisher.py**.
5. **[Optional]** To see the ROS structure open a new terminal and run the command: **rosrun rqt_graph rqt_graph**.
6. **[Optional]** To change the number of turtlebots in the simulation change the variable nbr_of_robots in variables/global_vars.py and add more robots in the launch file gazebo/launch/robots.launch by copying the code for one robot and change the robot number to 2, 3 and so on.


