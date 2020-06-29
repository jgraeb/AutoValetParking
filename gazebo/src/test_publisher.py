import rospy
import sys
from std_msgs.msg import Float32MultiArray

sys.path.append('../../') # enable importing modules from an upper directory:
from variables.global_vars import nbr_of_robots

"This is a test program for setting linear and rotational velocity to a selected turtlebot3"

pub = rospy.Publisher('robot_vel', Float32MultiArray, queue_size=1)
rospy.init_node('test_node')

vel_command = [0, 0, 0] #[robot_nbr, linear_vel, rotational_vel]

while not rospy.is_shutdown():
    robot_nbr = float(input("Choose robot number between 0 and {}\n".format(nbr_of_robots-1)))
    while robot_nbr < 0 or robot_nbr >= nbr_of_robots:
        robot_nbr = float(input("Robot nbr doesn't exist. Coose from robot 0 - {}\n".format(nbr_of_robots-1)))
    
    vel_command[0] = robot_nbr
    vel_command[1] = float(input("Set linear velocity [m/s] \n"))
    vel_command[2] = float(input("Set rotational velocity [rad/s] \n"))
    
    vel_topic = Float32MultiArray()
    vel_topic.data = vel_command
    
    pub.publish(vel_topic)
