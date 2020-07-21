# Automated Valet Parking - Robot Communication
# Tom Andersson
# California Institute of Technology
# June, 2020

import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import nbr_of_robots

pub_rate = 100 #[Hz]


def set_velocity(vel_topic):
    """This function takes a ros topic containing a Float32MultiArray with robot nbr as the first element,
    linear velocity as the second argument and rotational velocity as the last. 
    The function sets this data into 2 twist objects, defined globaly in a list."""
    twists[int(vel_topic.data[0])].linear.x = vel_topic.data[1]
    twists[int(vel_topic.data[0])].angular.z = vel_topic.data[2]


def publish_velocities(twists, pub):
    """This functions takes a tuple of publishers, pub, and a tuple of Twists, twist,
    and publishes each twist with the corresponding publisher. """
    for i, _ in enumerate(pub):
        pub[i].publish(twists[i])


def publish_robot_state(robot_nbr):
    """For a given robot number, this function extracts the robot states from the stored Odometry objects and thereafter publish the states.
    The states consists of x and y position [m],
    the yaw angle ie the rotation around z-axis starting from positive x-axis e(-pi - pi) [rad] (converted from quaternions),
    the linear and rotational velocity [m/s] [rad/s]."""
    nbr = robot_nbr.data
    state_topic = Float32MultiArray()
    
    x_pos = robot_states[nbr].pose.pose.position.x
    y_pos = robot_states[nbr].pose.pose.position.y

    quaternion_orient = Rotation.from_quat([robot_states[nbr].pose.pose.orientation.x,
                                            robot_states[nbr].pose.pose.orientation.y,
                                            robot_states[nbr].pose.pose.orientation.z,
                                            robot_states[nbr].pose.pose.orientation.w])
    euler_orient = quaternion_orient.as_euler('xyz')
    yaw = euler_orient[2]

    x_vel = robot_states[nbr].twist.twist.linear.x
    rot_vel = robot_states[nbr].twist.twist.angular.z

    state_topic.data = [x_pos, y_pos, yaw, x_vel, rot_vel]
    pub_robot_state.publish(state_topic)


def publish_model_state(model_name):
    """For a given model_name, this function extracts the model states from the stored ModelStates.pose and ModelStates.twist objects and thereafter publish the states."
    The states consists of x and y position [m],
     the yaw angle ie. the rotation around z-axis starting from positive x-axis e(-pi - pi) [rad] (converted from quaternions),
    the velocity in the x-axis and y-axis [m/s] and and rotational velocity [rad/s]"""
    name = model_name.data
    state_topic = Float32MultiArray()
    x_pos = model_states[name][0].position.x
    y_pos = model_states[name][0].position.y

    quaternion_orient = Rotation.from_quat([model_states[name][0].orientation.x,
                                            model_states[name][0].orientation.y,
                                            model_states[name][0].orientation.z,
                                            model_states[name][0].orientation.w])
    euler_orient = quaternion_orient.as_euler('xyz')
    yaw = euler_orient[2]
    x_vel = model_states[name][1].linear.x
    y_vel = model_states[name][1].linear.y
    rot_vel = model_states[name][1].angular.z
    state_topic.data = [x_pos, y_pos, yaw, x_vel, y_vel, rot_vel]
    pub_model_state.publish(state_topic)


def update_robot_states(odom, robot_nbr):
    "For a given robor number, this function stores the received Odometry object"
    robot_states[robot_nbr] = odom


def update_model_states(states):
    """This function stores the ModelStates.pose and ModelStates.twist objects in a dictionary with the corresponding model name as key.
    Data for special models such as walls and grpund plane are not stored."""
    for model, name  in enumerate(states.name):
        if name not in spec_model_names:
            model_states[name] = [states.pose[model], states.twist[model]]


"""This program creates a list of Twist objects, one object for each robot.
The program publishes the twist objects as velocity commands (linear and rotational) to the robots at a certain rate.
The data is updated as soon as a topic with new data arrives. 
This program also subscribes to a node for each robot that publishes the odemetry of the robot.
Upon request through another node the program is subscribed to, the program will publish the relevant data in this odemetry,
i.e the x and y position, the orientation and the linear and rotational velocity

Additionally this program is subscribed to the topic that publishes the model states at a certian rate.
Upon request through another node the program is subscribed to, the program will publish the relevant data of the model states
i.e the x and y position, the orientation, the velocity in the x- and y-axis and the rotational velocity"""


twists = [Twist() for i in range(nbr_of_robots)]
for i in range(nbr_of_robots):
    twists[i].linear.x = 0
    twists[i].linear.y = 0
    twists[i].linear.z = 0
    twists[i].angular.x = 0
    twists[i].angular.y = 0
    twists[i].angular.z = 0

robot_states = [[0, 0, 0, 0, 0] for i in range(nbr_of_robots)] #[pos_x, pos_y, orientation, lin_vel, rot_vel]
model_states = {} #{model_name: [pos_x, pos_y, orientation, vel_x, vel_y, rot_vel]}
spec_model_names = ('walls', 'parking_lot_ground_plane')

rospy.init_node("gazebo_com")

for i in range(nbr_of_robots):
    rospy.Subscriber('Robot{}/odom'.format(i), Odometry, update_robot_states, i)

rospy.Subscriber('robot_set_vel', Float32MultiArray, set_velocity)
rospy.Subscriber('robot_state_request', UInt16, publish_robot_state)
rospy.Subscriber('gazebo/model_states', ModelStates, update_model_states)
rospy.Subscriber('model_state_request', String, publish_model_state)

pub_vel = tuple(rospy.Publisher('Robot{}/cmd_vel'.format(i), Twist, queue_size = 1) for i in range(nbr_of_robots))
pub_robot_state = rospy.Publisher('robot_state', Float32MultiArray, queue_size = 1)
pub_model_state = rospy.Publisher('model_state', Float32MultiArray, queue_size = 1)

r = rospy.Rate(pub_rate)

while not rospy.is_shutdown():
    publish_velocities(twists, pub_vel)
    r.sleep()