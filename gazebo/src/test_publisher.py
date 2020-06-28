import rospy
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('robot_speed', Float32MultiArray, queue_size=10)
rospy.init_node('test_node')

speed_command = [0, 0, 0] #[robot_nbr, linear_speed, rotational_speed]

while not rospy.is_shutdown():
    speed_command[0] = float(input("Choose robot\n"))
    speed_command[1] = float(input("Set linear speed\n"))
    speed_command[2] = float(input("Set rotational speed\n"))
    
    speed_topic = Float32MultiArray()
    speed_topic.data = speed_command
    
    pub.publish(speed_topic)
