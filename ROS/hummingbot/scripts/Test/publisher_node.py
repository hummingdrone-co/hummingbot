#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş

import rospy
from sensor_msgs.msg import Joy

NODE_NAME = "test_publisher"
TOPIC_NAME = "/joy"
AXES_MSG = [1.0, 1.0]
BUTTON_MSG = [0, 0, 0, 1]
# stop button is button_msg[3]. when you push that button
# button_msg[3] should be equal to 1.
# when stop button is pushed, motors should stop.

if __name__ == "__main__":

    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(TOPIC_NAME, Joy, queue_size=10)
    rate = rospy.Rate(2)
    message = Joy()

    while not rospy.is_shutdown():
        message.axes = AXES_MSG
        message.buttons = BUTTON_MSG
        pub.publish(message)
        rate.sleep()

    rospy.loginfo("Node has stopped!")
