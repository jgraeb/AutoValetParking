# Automated Valet Parking - Robot Control
# Tom Andersson
# California Institute of Technology
# June, 2020

import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import nbr_of_robots

class RobotCtrl:
    """This class is used for setting velocities and reading the states of the robots in the gazebo simulation.
    This is done through subscribed and published ros topics between this class and the robot_communication program.
    Topics are only published and received opon request."""
    
    def __init__(self):
        "Initializes the ros node, sets up subscribed and published topics and creates variables for storing the data."
        #rospy.init_node('robot_ctrl')        
        self.pub_vel = rospy.Publisher('robot_set_vel', Float32MultiArray, queue_size=1)
        self.pub_state_request = rospy.Publisher('robot_state_request', UInt16, queue_size=1)
        rospy.Subscriber('robot_state', Float32MultiArray)

        self.vel_commands = [[0, 0] for i in range(nbr_of_robots)]
        self.robot_states = [[0, 0, 0, 0, 0] for i in range(nbr_of_robots)] #[pos_x, pos_y, orientation, lin_vel, rot_vel]
        self.vel_topic = Float32MultiArray()


    def set_lin_vel(self, robot_nbr, lin_vel):
        "Sets a new linear velocity for a given robot number. [m/s]"
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.vel_commands[robot_nbr][0] = lin_vel
        self.__publish_vel(robot_nbr)


    def set_rot_vel(self, robot_nbr, rot_vel):
        "Sets a new rotational velocity for a given robot number. [rad/s]"
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.vel_commands[robot_nbr][1] = rot_vel
        self.__publish_vel(robot_nbr)  


    def set_vel(self, robot_nbr, lin_vel, rot_vel):
        "Sets a new linear and rotational velocity for a given robot number. [m/s], [rad/s]"
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.vel_commands[robot_nbr][0] = lin_vel
        self.vel_commands[robot_nbr][1] = rot_vel
        self.__publish_vel(robot_nbr)


    def __publish_vel(self, robot_nbr):
        "Publishes a new velocity for a given robot number. [m/s], [rad/s]"
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.vel_topic.data = [float(robot_nbr), float(self.vel_commands[robot_nbr][0]), float(self.vel_commands[robot_nbr][1])]
        self.pub_vel.publish(self.vel_topic)

    
    def __update_state(self, robot_nbr):
        self.pub_state_request.publish(robot_nbr)
        self.robot_states[robot_nbr] = rospy.wait_for_message('robot_state', Float32MultiArray).data

    
    def get_pos(self, robot_nbr):
        """Returns the x, y position, [m], as well as the orientation around z-axis starting from positive x-axis,
        e(-pi - pi) [rad], for a given robot number. (x_pos, y_pos, orientation)"""
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.__update_state(robot_nbr)
        return (self.robot_states[robot_nbr][0], self.robot_states[robot_nbr][1], self.robot_states[robot_nbr][2])


    def get_vel(self, robot_nbr):
        "Returns the linear and rorational velocity for a given robot number. (lin_vel, rot_vel) [m/s], [rad/s]"
        if robot_nbr < 0 or robot_nbr >= nbr_of_robots:
            raise Exception("Invalid robot number")
        self.__update_state(robot_nbr)
        return (self.robot_states[robot_nbr][3], self.robot_states[robot_nbr][4])