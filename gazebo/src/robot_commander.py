
import rospy
import sys
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import nbr_of_robots

def set_velocity(vel_topic):
    """This function takes a ros topic containing a Float32MultiArray with robot nbr as the first element,
    linear velocity as the second argument and rotational velocity as the last. 
    The function sets this data into 2 twist objects, defined globaly in a list."""
    twist[int(vel_topic.data[0])].linear.x = vel_topic.data[1]
    twist[int(vel_topic.data[0])].angular.z = vel_topic.data[2]

def publish_velocities(twist, pub):
    """This functions takes a tuple of publishers, pub, and a tuple of Twists, twist,
    and publishes each twist with the corresponding publisher. """
    for i, _ in enumerate(pub):
        pub[i].publish(twist[i])

"""This program creates a list of Twist objects, one object for each robot.
The program publishes the twist objects to the robots at a certain rate.
The data is updated as soon as a topic with new data arrives."""
pub_rate = 100 #Hz
nbr_of_robots = 2
twist = [Twist() for i in range(nbr_of_robots)]

for i in range(nbr_of_robots):
    twist[i].linear.x = 0
    twist[i].linear.y = 0
    twist[i].linear.z = 0
    twist[i].angular.x = 0
    twist[i].angular.y = 0
    twist[i].angular.z = 0

rospy.init_node("robot_commander")
rospy.Subscriber('robot_vel', Float32MultiArray, set_velocity)
pub = tuple(rospy.Publisher('robot{}/cmd_vel'.format(i), Twist, queue_size = 1) for i in range(nbr_of_robots))
r = rospy.Rate(pub_rate)

while not rospy.is_shutdown():
    publish_velocities(twist, pub)
    r.sleep()

