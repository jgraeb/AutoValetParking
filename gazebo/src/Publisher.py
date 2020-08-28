# Automated Valet Parking - Publisher
# Tom Andersson
# California Institute of Technology
# August, 2020

import rospy
from std_msgs.msg import Float32


class Publisher:
    "This class is used for publishing rostopics relevant for plotting"
    
    def __init__(self, start_x, start_y, start_yaw):
        self.pub_float = rospy.Publisher('ex_float', Float32, queue_size=1)

        self.pub_state_x = rospy.Publisher('state_x', Float32, queue_size=1)
        self.pub_state_y = rospy.Publisher('state_y', Float32, queue_size=1)
        self.pub_state_v = rospy.Publisher('state_v', Float32, queue_size=1)
        self.pub_state_yaw = rospy.Publisher('state_yaw', Float32, queue_size=1)
        self.pub_pred_x = rospy.Publisher('pred_x', Float32, queue_size=1)
        self.pub_pred_y = rospy.Publisher('pred_y', Float32, queue_size=1)
        self.pub_pred_v = rospy.Publisher('pred_v', Float32, queue_size=1)
        self.pub_pred_yaw = rospy.Publisher('pred_yaw', Float32, queue_size=1)
        self.pub_error_x = rospy.Publisher('error_x', Float32, queue_size=1)
        self.pub_error_y = rospy.Publisher('error_y', Float32, queue_size=1)
        self.pub_error_v = rospy.Publisher('error_v', Float32, queue_size=1)
        self.pub_error_yaw = rospy.Publisher('error_yaw', Float32, queue_size=1)
        self.pub_relative_error_x = rospy.Publisher('relative_error_x', Float32, queue_size=1)
        self.pub_relative_error_y = rospy.Publisher('relative_error_y', Float32, queue_size=1)
        self.pub_relative_error_v = rospy.Publisher('relative_error_v', Float32, queue_size=1)
        self.pub_relative_error_yaw = rospy.Publisher('relative_error_yaw', Float32, queue_size=1)
        
        self.last_x = start_x
        self.last_y = start_y
        self.last_v = 0
        self.last_yaw = start_yaw
    
    def publish_one_step_pred_error(self,state_x, state_y, state_v, state_yaw, pred_x, pred_y, pred_v, pred_yaw):
        error_x = state_x - pred_x
        error_y = state_y - pred_y
        error_v = state_v - pred_v
        error_yaw = state_yaw - pred_yaw
        
        self.pub_state_x.publish(state_x)
        self.pub_state_y.publish(state_y)
        self.pub_state_v.publish(state_v)
        self.pub_state_yaw.publish(state_yaw)

        self.pub_pred_x.publish(pred_x)
        self.pub_pred_y.publish(pred_y)
        self.pub_pred_v.publish(pred_v)
        self.pub_pred_yaw.publish(pred_yaw)

        self.pub_error_x.publish(error_x)
        self.pub_error_y.publish(error_y)
        self.pub_error_v.publish(error_v)
        self.pub_error_yaw.publish(error_yaw)

        try:
            relative_error_x = error_x/(self.last_x-state_x)
            relative_error_y = error_y/(self.last_y-state_y)
            relative_error_v = error_v/(self.last_v-state_v)
            relative_error_yaw = error_yaw/(self.last_yaw-state_yaw)

            self.pub_relative_error_x.publish(relative_error_x)
            self.pub_relative_error_y.publish(relative_error_y)
            self.pub_relative_error_v.publish(relative_error_v)
            self.pub_relative_error_yaw.publish(relative_error_yaw)
            
        except ZeroDivisionError:
            print("No state change, relative error not calculated.")
        
        self.last_x = state_x
        self.last_y = state_y
        self.last_v = state_v
        self.last_yaw = state_yaw

    def publish_float(self, value):
        self.pub_float.publish(value)
