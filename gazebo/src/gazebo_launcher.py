# Automated Valet Parking - Gazebo Launcher
# Tom Andersson
# California Institute of Technology
# July, 2020

import roslaunch
import rospy
import sys
from std_msgs.msg import Float32MultiArray

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import catkin_ws_folder, nbr_of_robots, nbr_of_pedestrians, nbr_of_obstacles


def gazebo_ctrl(launch_topic):
    """When a topic is received, this function will set variables to declare if the gazebo world should be launched,
    terminated or neither depending on the topic data and the current state of the world (if the world is already launched or not)."""
    global launching, terminate, launch_list
    if launch_topic.data != [0] and not launched:
        launch_list = launch_topic.data
        launching = True
    elif launched:
        terminate = True


"""This program is used to launch a Gazebo world and to launch robots, pedestrians and obstacles into that world.
The program subscribes to a rostopic that will publish commands telling this program to launch or terminate gazebo.
When a topic is received the function gazebo_ctrl determines if Gazebo should be launched or terminated.

If Gazebo shall be launched the program will first launch a world.
Thereafter it takes the topic and sort its content into 3 lists, robots, pedestians and obstacles. 
Each of these lists consists of 3 elements per object, x-pos, y-pos and yaw angle.
Each object is launched togeather with these arguments plus an identification number.

If Gazebo shall be terminated all previously launched files will be shutdown, 
launched objects are shutdown before the world itself is.
"""

launched = False
launching = False
terminate = False
launchers = []
launch_list = []
rospy.init_node("gazebo_launcher")
rospy.Subscriber('launch_gazebo', Float32MultiArray, gazebo_ctrl)


while not rospy.is_shutdown():
    if launching:

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_world = roslaunch.parent.ROSLaunchParent(uuid, [catkin_ws_folder+"/src/AutoValetParking/gazebo/launch/world.launch"])
        launch_world.start()
        launchers.append(launch_world)
        
        robots = launch_list[:nbr_of_robots*3]
        pedestrians = launch_list[nbr_of_robots*3:nbr_of_robots*3+nbr_of_pedestrians*3]
        obstacles = launch_list[nbr_of_robots*3+nbr_of_pedestrians*3:]

        for i in range(0, nbr_of_robots*3, 3):
            launch_folder = catkin_ws_folder+'/src/AutoValetParking/gazebo/launch/robot.launch'
            roslaunch_args = ['x:={}'.format(robots[i]), 'y:={}'.format(robots[i+1]), 'yaw:={}'.format(robots[i+2]), 'robot_nbr:={}'.format(int(i/3))]
            launchers.append(roslaunch.parent.ROSLaunchParent(uuid, [(launch_folder, roslaunch_args)]))
            launchers[-1].start()

        for i in range(0, nbr_of_pedestrians*3, 3):
            launch_folder = catkin_ws_folder+'/src/AutoValetParking/gazebo/launch/pedestrian.launch'
            roslaunch_args = ['x:={}'.format(pedestrians[i]), 'y:={}'.format(pedestrians[i+1]), 'yaw:={}'.format(pedestrians[i+2]), 'pedestrian_nbr:={}'.format(int(i/3))]
            launchers.append(roslaunch.parent.ROSLaunchParent(uuid, [(launch_folder, roslaunch_args)]))
            launchers[-1].start()

        for i in range(0, nbr_of_obstacles*3, 3):
            launch_folder = catkin_ws_folder+'/src/AutoValetParking/gazebo/launch/obstacle.launch'
            roslaunch_args = ['x:={}'.format(obstacles[i]), 'y:={}'.format(obstacles[i+1]), 'yaw:={}'.format(obstacles[i+2]), 'obstacle_nbr:={}'.format(int(i/3))]
            launchers.append(roslaunch.parent.ROSLaunchParent(uuid, [(launch_folder, roslaunch_args)]))
            launchers[-1].start()       

        launching = False
        launched = True

    elif terminate:
        terminate = False
        launched = False
        for launcher in reversed(launchers):
            launcher.shutdown()