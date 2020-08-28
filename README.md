# Automated Valet Parking


![](movies/replanning.gif)

An example of modular contract-based design using a directive/response architecture.

## Specific for the gazebo_ros branch:
This branch introduces gazebo simulations of turtlebot3 for the automated valet parking.
To run this branch you need to have the AutomatedValetParking package in a catkin workspace for ROS melodic.

### How to run a test for launching and controlling turtlebots, pedestrians and obstacles in a parking lot world in Gazebo:
0. Go into global_vars.py in the folder variables and change the variable "catkin_ws_folder" to match your location.
1. Open a new terminal and go into your catkin workspace and source the generated setup file with the command: **source devel/setup.bash**.
2. In the same terminal, go to the folder AutoValetParking/gazebo/src and run the gazebo launcher using the command: **python3 gazebo_launcher.py**.
3. Open an new terminal and go to the same folder: AutoValetParking/gazebo/src. Run the communication program using the command: **python3 communication.py**.
4. Again, open an new terminal and go to this folder: AutoValetParking/gazebo/src. Run the test program using the command **python3 test_robot_n_model_ctrl.py**
5. To test the different features, follow the instructions in the terminal of the test program.
6. **[Optional]** To see the ROS structure open a new terminal and run the command: **rosrun rqt_graph rqt_graph**.
7. **[Optional]** To change the number (or initial position) of turtlebots, pedestrians or obstacles in the simulation change the variable nbr_of_robots, nbr_of_pedestrians or nbr_of_obstacles in variables/global_vars.py and set new coordinates when launching the world in the code of the test program, test_robot_n_model_ctrl.py, using GazeboCtrl.launch_gazebo.

### Integration with the main simulations
To run the main simulations complete step 0-3 above and then run continous.py in the demo folder. The size of the environment have been reduced to fit the size of the turtlebots.

#### Trajectory control
The turtlebots were integrated by using its measured states instead of the predicted states, by giving the turtlebot the control outputs and by adding a "wait until" to achieve the correct sample time in track_async in car.py. The original model used acceleration and steering angle as control signals (accSteerCtrl). 3 Other models were derived as well, these uses the following control signals:
1. Velocity and steering angle (velCtrl).
2. Linear and rotational velocity (turtlebotCtrl).
3. Acceleration and rotational velocity (accRotVelCtrl).
The 3 new models were discretized using zero order hold and the full derivation will be available in the report in the documentation folder.

However the trajectory control is not yet working:
1. The computation time for the iterative_linear_mpc_control is both high and varying between samples. (Usually around 0.6 seconds but both longer and shorter times occur as well)
2. The one step prediction error is high. The error is worst for the velocity and, in the cases were the steering angle is used as control signal, the yaw angle. Using velocity and steering angle as control signals seem to give a higher error than using acceleration and rotational velocity but all alternative have high errors.

It was recommended that the high prediction error was due to an inaccurate model. Therefore, the 3 new models were created to see if another model would be able to overcome the problems mentioned above. Neither of the models solved the problems.
To find out if trio was causing the long computing times trio was taken out of track_async and/or iterative_linear_mpc_control but this did not seem to affect the computation time. As an effort to lower the prediction error, the sample period of the last iteration was used for the optimization instead of the variable tracking.DT, this did not solve the problem either.
The function update_model_state in mpc_starking.py was not using the proper system equations. This was changed but it did not seem to have a significant impact on the performance.
The predictions were tested by letting the system use the predicted states as the measured sates and only take the animated car into account. (See commented code in track_async in car.py) This worked for accSteerCtrl but not for the other models. Why it didn't work for the other models is still unclear.
It was recommended to try another solver instead, this has not yet been done due to time limitations. https://github.com/urosolia/MultiRate/blob/master/python/nonlinearFTOCP.py 



#### General tips.
1. Use the command **rqt_plot** to plot published ros topics in real time.
2. When changing the number of robots, pedestrians or objects that are being launched one must also change the corresponding variables in global_vars.py.
3. Sometimes gazebo doesn't shutdown properly but is still running in the background. This can cause problems when a new world is being launched. To eliminate this error run the command **killall gzserver** before launching a new world.
4. Sometimes a type error is shown when running gazebo/src/communication.py. The cause of this is yet unknown, just try to run it again if the error occurs.
5. Use the class Publisher together with **rqt_plot** to plot the prediction error. (See commented code in track_async in car.py) 






