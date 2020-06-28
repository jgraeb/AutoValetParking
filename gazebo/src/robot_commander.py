import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def set_velocity(speed_topic):
    twist[int(speed_topic.data[0])].linear.x = speed_topic.data[1]
    twist[int(speed_topic.data[0])].angular.z = speed_topic.data[2]

def publish_velocities(twist, pub):
        for i, _ in enumerate(pub):
            pub[i].publish(twist[i])


twist = [Twist(), Twist()]
for i, _ in enumerate(twist):
    twist[i].linear.x = 0
    twist[i].linear.y = 0
    twist[i].linear.z = 0
    twist[i].angular.x = 0
    twist[i].angular.y = 0
    twist[i].angular.z = 0

rospy.init_node("robot_commander")
rospy.Subscriber('robot_speed', Float32MultiArray, set_velocity)
pub = (rospy.Publisher('robot0/cmd_vel', Twist, queue_size = 1),
        rospy.Publisher('robot1/cmd_vel', Twist, queue_size = 1)
        )
r = rospy.Rate(100) # 100hz, T=10ms

while(1):
    publish_velocities(twist, pub)
    r.sleep()

