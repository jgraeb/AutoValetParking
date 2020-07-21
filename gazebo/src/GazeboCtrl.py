# Automated Valet Parking - Gazebo Control
# Tom Andersson
# California Institute of Technology
# July, 2020

import rospy
from std_msgs.msg import Float32MultiArray


class GazeboCtrl: 
    "This class is used to publish a rostopic for launching/terminating a gazebo world with robots, pedestrians and obstacles at given positions"
    
    def __init__(self):
        "Creates a ros publisher"
        self.pub_gazebo_command = rospy.Publisher('launch_gazebo', Float32MultiArray, queue_size=1, latch=True)
        self.launch_topic = Float32MultiArray()
    
    def launch_gazebo(self, robots=[], pedestrians=[], obstacles=[]):
        """Takes 3 lists as arguments, robots, pedestrians and obstacles containing the x- and y-pos and the yaw angle for each object
        (Ie. each list can be empty, can consist of 3 floats or can be a list of lists depending on how many of each object that are going to be launched).
        The 3 lists are converted into one 1-dimensional list to and is thereafter published"""
        launch_list = [] 
        for i in robots:
            if type(i) == list:
                for value in i:
                    launch_list.append(value)
            else:
                launch_list.append(i)
        for i in pedestrians:
            if type(i) == list:
                for value in i:
                    launch_list.append(value)
            else:
                launch_list.append(i)
        for i in obstacles:
            if type(i) == list:
                for value in i:
                    launch_list.append(value)
            else:
                launch_list.append(i)        

        self.launch_topic.data = launch_list
        self.pub_gazebo_command.publish(self.launch_topic)

    def terminate_gazebo(self):
        "Publishes [0] which tells the gazebo_launcher to terminate."
        self.launch_topic.data = [0]
        self.pub_gazebo_command.publish(self.launch_topic)
