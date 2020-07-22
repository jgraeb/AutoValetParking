# Automated Valet Parking


![](movies/replanning.gif)

An example of modular contract-based design using a directive/response architecture.

## Specific for the gazebo_ros branch:
This branch introduces gazebo simulations of turtlebot3 for the automated valet parking. The functionallity and this README are currently under construction.
To run this branch you need to have the AutomatedValetParking package in a catkin workspace for ROS melodic.

### How to run a test for launching and controlling turtlebots, pedestrians and obstacles in a parking lot world in Gazebo:
1. Open a new terminal and go into your catkin workspace and source the generated setup file with the command: **source devel/setup.bash**.
2. In the same terminal, go to the folder AutoValetParking/gazebo/src and run the gazebo launcher using the command: **python3 gazebo_launcher.py**.
3. Open an new terminal and go to the same folder: AutoValetParking/gazebo/src. Run the communication program using the command: **python3 communication.py**.
4. Again, open an new terminal and go to this folder: AutoValetParking/gazebo/src. Run the test program using the command **python3 test_robot_n_model_ctrl.py**
5. To test the different features, follow the instructions in the terminal of the test program.
6. **[Optional]** To see the ROS structure open a new terminal and run the command: **rosrun rqt_graph rqt_graph**.
7. **[Optional]** To change the number (or initial position) of turtlebots, pedestrians or obstacles in the simulation change the variable nbr_of_robots, nbr_of_pedestrians or nbr_of_obstacles in variables/global_vars.py and set new coordinates when launching the world in the code of the test program, test_robot_n_model_ctrl.py, using GazeboCtrl.launch_gazebo.


