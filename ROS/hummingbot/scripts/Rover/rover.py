#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : Hasan Basri Aykol [basri@hummingdrone.co],
#          Mustafa Durmuş    [mustafa@hummingdrone.co]

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

NODE_NAME = "rover"
SUB_TOPIC_NAME = "/control"
PUB_TOPIC_NAMES = ['/gazebo_hummingbot_client/left_vel','/gazebo_hummingbot_client/right_vel']


def calculate_velocity(linear,angular):
    """
    calculates speed of the wheels and returns them.
 
    Formula:
    left  wheel velocity : twist.linear.x - ( twist.angular.z / 2 ) 
    right wheel velocity : twist.linear.x + ( twist.angular.z / 2 )
    
    Parameters:
    linear  (float32) : value of twist.linear.x
    angular (float32) : value of twist.angular.z
    
    Returns:
    float32,float32
    """
    left_wheel  = linear - (angular / 2)
    right_wheel = linear + (angular / 2)

    return left_wheel,right_wheel


def callback_create_publisher(twist):
    """
    publishes speeds of the wheels.

    Parameters:
    twist : the message of teleop node
    """
    msg_left.data, msg_right.data = calculate_velocity(twist.linear.x,twist.angular.z)

    pub_left.publish(msg_left.data)
    pub_right.publish(msg_right.data)


if __name__ == '__main__':
    
    # node created
    rospy.init_node(NODE_NAME)
    
    # publishers created
    pub_left  = rospy.Publisher(PUB_TOPIC_NAMES[0],Float32,queue_size = 10)
    pub_right = rospy.Publisher(PUB_TOPIC_NAMES[1],Float32,queue_size = 10)
    
    # message types created
    msg_left  = Float32()
    msg_right = Float32()
    
    #subscriber created
    sub = rospy.Subscriber(SUB_TOPIC_NAME,Twist,callback_create_publisher)
    
    rospy.spin()
    
