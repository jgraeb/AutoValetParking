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
        self.pedestrian_names = ['pedestrian{}'.format(i) for i in range(nbr_of_pedestrians)]
        self.obstacle_names = ['obstacle{}'.format(i) for i in range(nbr_of_obstacles)]
        self.pubs_vel = dict(zip(self.pedestrian_names, [rospy.Publisher('{}_set_vel'.format(name), Float32MultiArray, queue_size=1) for name in self.pedestrian_names]))
        
        self.pub_state_request = rospy.Publisher('model_state_request', String, queue_size=1)

        rospy.Subscriber('model_state', Float32MultiArray)
        
        self.vel_commands = dict.fromkeys(self.pedestrian_names, [0, 0, 0])
        self.model_states = {} #{model_name: [pos_x, pos_y, orientation, vel_x, vel_y, rot_vel]}
        self.vel_topic = Float32MultiArray()

    def set_x_vel(self, model_name, x_vel):
        "Sets a new x velocity for a given model name. [m/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_commands[model_name][0] = x_vel
        self.__publish_vel(model_name)

    def set_y_vel(self, model_name, y_vel):
        "Sets a new y velocity for a given model name. [m/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_commands[model_name][1] = y_vel
        self.__publish_vel(model_name)

    def set_lin_vel(self, model_name, x_vel, y_vel):
        "Sets a new linear velocity for a given model name. [m/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_commands[model_name][0] = x_vel
        self.vel_commands[model_name][1] = y_vel
        self.__publish_vel(model_name)


    def set_rot_vel(self, model_name, rot_vel):
        "Sets a new rotational velocity for a given model name. [rad/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_commands[model_name][2] = rot_vel
        self.__publish_vel(model_name)


    def set_vel(self, model_name, x_vel, y_vel, rot_vel):
        "Sets a new linear and rotational velocity for a given model name. [m/s], [rad/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_commands[model_name][0] = x_vel
        self.vel_commands[model_name][1] = y_vel
        self.vel_commands[model_name][2] = rot_vel
        self.__publish_vel(model_name)


    def __publish_vel(self, model_name):
        "Publishes a new velocity for a given model name. [m/s], [rad/s]"
        if model_name not in self.pedestrian_names:
            raise Exception("Invalid model name, only the pedestrians are moavable.")
        self.vel_topic.data = [float(self.vel_commands[model_name][0]), float(self.vel_commands[model_name][1]), float(self.vel_commands[model_name][2])]
        self.pubs_vel[model_name].publish(self.vel_topic)


    def __update_state(self, model_name):
        self.pub_state_request.publish(model_name)
        self.model_states[model_name] = rospy.wait_for_message('model_state', Float32MultiArray).data

    
    def get_pos(self, model_name):
        """Returns the x, y position, [m], as well as the orientation around z-axis starting from positive x-axis,
        e(-pi - pi) [rad], for a given model name. (x_pos, y_pos, orientation)"""
        if model_name not in self.pedestrian_names and model_name not in self.obstacle_names:
            raise Exception("Invalid model name.")
        self.__update_state(model_name)
        return (self.model_states[model_name][0], self.model_states[model_name][1], self.model_states[model_name][2])


    def get_vel(self, model_name):
        "Returns the linear and rorational velocity for a given model name. (x_vel, y_vel, rot_vel) [m/s], [rad/s]"
        if model_name not in self.pedestrian_names and model_name not in self.obstacle_names:
            raise Exception("Invalid model name.")
        self.__update_state(model_name)
        return (self.model_states[model_name][3], self.model_states[model_name][4], self.model_states[model_name][5])