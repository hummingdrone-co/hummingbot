#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş [mustafa@hummingdrone.co]

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

NODE_NAME = 'joy_teleop'
SUB_TOPIC_NAME = '/joy'
PUB_TOPIC_NAME = '/control'

def callback(data):
    """
    Receives joystick messages (subscribed to Joy topic).
    then converts the joysick inputs into Twist commands.

    axis 1 aka left stick vertical controls linear speed.

    axis 0 aka left stick horizonal controls angular speed.
    """
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[0]
    pub.publish(twist)


# This ROS Node converts Joystick inputs from the joy node into twist message.
if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo("Node initialized!")

    pub = rospy.Publisher(PUB_TOPIC_NAME, Twist, queue_size=10)
    rospy.loginfo("Waiting for Joy message!")

    rospy.Subscriber(SUB_TOPIC_NAME, Joy, callback)
    rospy.spin()
