#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors : Hasan Basri Aykol [basri@hummingdrone.co],
#           Mustafa Durmuş    [mustafa@hummingdrone.co]

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse


NODE_NAME = "rover"
SUB_TOPIC_NAME = "/control"
VELOCITY_TOPIC_PARAMS = ['velocity_topic_left', 'velocity_topic_right']
VELOCITY_COEF_PARAM = 'velocity_coef'
SRV_TOPIC_NAME = "/stop"


def handler_stop_service(req):
    """ stops wheels by publishing 0 velocity."""
    pub_left.publish(0)
    pub_right.publish(0)
    return EmptyResponse()

def calculate_velocity(linear, angular):
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
    left_wheel  = vel_coef * (linear - (angular / 2))
    right_wheel = vel_coef * (linear + (angular / 2))

    return left_wheel, right_wheel


def callback_create_publisher(twist):
    """
    publishes speeds of the wheels.

    Parameters:
    twist : the message of teleop node
    """
    msg_left.data, msg_right.data = calculate_velocity(twist.linear.x, twist.angular.z)

    pub_left.publish(msg_left.data)
    pub_right.publish(msg_right.data)


if __name__ == '__main__':

    # node created
    rospy.init_node(NODE_NAME)

    # read velocity topics and coefficient parameter
    vel_coef = rospy.get_param(VELOCITY_COEF_PARAM)
    vel_topic_left = rospy.get_param(VELOCITY_TOPIC_PARAMS[0])
    vel_topic_right = rospy.get_param(VELOCITY_TOPIC_PARAMS[1])

    # publishers created
    pub_left  = rospy.Publisher(vel_topic_left, Float32, queue_size = 10)
    pub_right = rospy.Publisher(vel_topic_right, Float32, queue_size = 10)

    # message types created
    msg_left  = Float32()
    msg_right = Float32()

    # subscriber created
    sub = rospy.Subscriber(SUB_TOPIC_NAME, Twist, callback_create_publisher)

    # service created
    rospy.Service(SRV_TOPIC_NAME, Empty, handler_stop_service)

    rospy.spin()
