# Automated Valet Parking - Robot Communication
# Tom Andersson
# California Institute of Technology
# June, 2020

import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist

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


def publish_state(robot_nbr):
    "This function publishes the states for a given robot number (see the function update_states for a definition of the states)."
    state_topic = Float32MultiArray()
    state_topic.data = robot_states[robot_nbr.data]
    pub_state.publish(state_topic)


def update_states(odom, robot_nbr):
    """For a given robor number, this function stores the x and y position [m],
    the rotation around z-axis starting from positive x-axis e(-pi - pi) [rad] (converted from quaternions),
    the linear and rotational velocity [m/s], from a given Odometry object."""
    robot_states[robot_nbr][0] = odom.pose.pose.position.x
    robot_states[robot_nbr][1] = odom.pose.pose.position.y

    quaternion_orient = Rotation.from_quat([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
    euler_orient = quaternion_orient.as_euler('xyz')
    robot_states[robot_nbr][2] = euler_orient[2]

    robot_states[robot_nbr][3] = odom.twist.twist.linear.x
    robot_states[robot_nbr][4] = odom.twist.twist.angular.z


"""This program creates a list of Twist objects, one object for each robot.
The program publishes the twist objects as velocity commands (linear and rotational) to the robots at a certain rate.
The data is updated as soon as a topic with new data arrives. 
This program also subscribes to a node for each robot that publishes the odemetry of the robot.
Upon request through another node the program is subscribed to, the program will publish the relevant data in this odemetry,
i.e the x and y position, the orientation and the linear and rotational velocity"""

twists = [Twist() for i in range(nbr_of_robots)]
for i in range(nbr_of_robots):
    twists[i].linear.x = 0
    twists[i].linear.y = 0
    twists[i].linear.z = 0
    twists[i].angular.x = 0
    twists[i].angular.y = 0
    twists[i].angular.z = 0

robot_states = [[0, 0, 0, 0, 0] for i in range(nbr_of_robots)] #[pos_x, pos_y, orientation, lin_vel, rot_vel]

rospy.init_node("robot_com")

for i in range(nbr_of_robots):
    rospy.Subscriber('robot{}/odom'.format(i), Odometry, update_states, i)

rospy.Subscriber('robot_set_vel', Float32MultiArray, set_velocity)
rospy.Subscriber('robot_state_request', UInt16, publish_state)

pub_vel = tuple(rospy.Publisher('robot{}/cmd_vel'.format(i), Twist, queue_size = 1) for i in range(nbr_of_robots))
pub_state = rospy.Publisher('robot_state', Float32MultiArray, queue_size = 1)
r = rospy.Rate(pub_rate)

while not rospy.is_shutdown():
    publish_velocities(twists, pub_vel)
    r.sleep()