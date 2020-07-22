# Automated Valet Parking - Model Control
# Tom Andersson
# California Institute of Technology
# July, 2020

import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import nbr_of_pedestrians, nbr_of_obstacles


class ModelCtrl:
    """This class is used for setting velocities and reading the states of the models in the gazebo simulation.
    This is done through subscribed and published ros topics between this class and the model plugins movemodel.
    Topics are only published and received opon request."""
    
    def __init__(self):
        "Initializes the ros node, sets up subscribed and published topics and creates variables for storing the data."
        self.pubs_vel = [rospy.Publisher('pedestrian{}_set_vel'.format(i), Float32MultiArray, queue_size=1) for i in range(nbr_of_pedestrians)]
        self.pub_state_request = rospy.Publisher('model_state_request', String, queue_size=1)

        rospy.Subscriber('model_state', Float32MultiArray)
        
        self.vel_commands = [[0, 0, 0] for i in range(nbr_of_pedestrians)]
        self.pedestrian_states = [[0, 0, 0, 0, 0, 0] for i in range(nbr_of_pedestrians)] #[pos_x, pos_y, orientation, vel_x, vel_y, rot_vel]
        self.obstacle_states = [[0, 0, 0] for i in range(nbr_of_obstacles)] #[pos_x, pos_y, orientation]
        self.vel_topic = Float32MultiArray()

    def set_x_vel(self, pedestrian_nbr, x_vel):
        "Sets a new x velocity for a given pedestrian number. [m/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_commands[pedestrian_nbr][0] = x_vel
        self.__publish_vel(pedestrian_nbr)

    def set_y_vel(self, pedestrian_nbr, y_vel):
        "Sets a new y velocity for a given pedestrian number. [m/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_commands[pedestrian_nbr][1] = y_vel
        self.__publish_vel(pedestrian_nbr)

    def set_lin_vel(self, pedestrian_nbr, x_vel, y_vel):
        "Sets a new linear velocity for a given pedestrian_nbr. [m/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_commands[pedestrian_nbr][0] = x_vel
        self.vel_commands[pedestrian_nbr][1] = y_vel
        self.__publish_vel(pedestrian_nbr)


    def set_rot_vel(self, pedestrian_nbr, rot_vel):
        "Sets a new rotational velocity for a given pedestrian_nbr. [rad/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_commands[pedestrian_nbr][2] = rot_vel
        self.__publish_vel(pedestrian_nbr)


    def set_vel(self, pedestrian_nbr, x_vel, y_vel, rot_vel):
        "Sets a new linear and rotational velocity for a given pedestrian_nbr. [m/s], [rad/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_commands[pedestrian_nbr] = [x_vel, y_vel, rot_vel]
        self.__publish_vel(pedestrian_nbr)


    def __publish_vel(self, pedestrian_nbr):
        "Publishes a new velocity for a given pedestrian_nbr. [m/s], [rad/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.vel_topic.data = [float(self.vel_commands[pedestrian_nbr][0]), float(self.vel_commands[pedestrian_nbr][1]), float(self.vel_commands[pedestrian_nbr][2])]
        self.pubs_vel[pedestrian_nbr].publish(self.vel_topic)


    def __update_pedestrian_state(self, pedestrian_nbr):
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.pub_state_request.publish('pedestrian{}'.format(pedestrian_nbr))
        self.pedestrian_states[pedestrian_nbr] = rospy.wait_for_message('model_state', Float32MultiArray).data

    def __update_obstacle_state(self, obstacle_nbr):
        if obstacle_nbr < 0 or obstacle_nbr >= nbr_of_obstacles:
            raise Exception("Invalid obstacle number")
        self.pub_state_request.publish('obstacle{}'.format(obstacle_nbr))
        self.obstacle_states[obstacle_nbr] = rospy.wait_for_message('model_state', Float32MultiArray).data
    
    def get_pedestrian_pos(self, pedestrian_nbr):
        """Returns the x, y position, [m], as well as the orientation around z-axis starting from positive x-axis,
        e(-pi - pi) [rad], for a given pedestrian_nbr. (x_pos, y_pos, orientation)"""
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.__update_pedestrian_state(pedestrian_nbr)
        return (self.pedestrian_states[pedestrian_nbr][0], self.pedestrian_states[pedestrian_nbr][1], self.pedestrian_states[pedestrian_nbr][2])

    def get_obstacle_pos(self, obstacle_nbr):
        """Returns the x, y position, [m], as well as the orientation around z-axis starting from positive x-axis,
        e(-pi - pi) [rad], for a given obstacle_nbr. (x_pos, y_pos, orientation)"""
        if obstacle_nbr < 0 or obstacle_nbr >= nbr_of_obstacles:
            raise Exception("Invalid obstacle number")
        self.__update_obstacle_state(obstacle_nbr)
        return (self.obstacle_states[obstacle_nbr][0], self.obstacle_states[obstacle_nbr][1], self.obstacle_states[obstacle_nbr][2])

    def get_vel(self, pedestrian_nbr):
        "Returns the linear and rorational velocity for a given pedestrian_nbr. (x_vel, y_vel, rot_vel) [m/s], [rad/s]"
        if pedestrian_nbr < 0 or pedestrian_nbr >= nbr_of_pedestrians:
            raise Exception("Invalid pedestrian number")
        self.__update_pedestrian_state(pedestrian_nbr)
        return (self.pedestrian_states[pedestrian_nbr][3], self.pedestrian_states[pedestrian_nbr][4], self.pedestrian_states[pedestrian_nbr][5])