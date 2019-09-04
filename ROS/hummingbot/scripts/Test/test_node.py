#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş [mustafa@hummingdrone.co]

import time
import unittest
import rosunit
import rospy
from std_msgs.msg import Float32

PKG = 'hummingbot'
NODE_NAME = "test"
SUB_TOPIC_NAMES = ["/gazebo_hummingbot_client/left_vel",
                   "/gazebo_hummingbot_client/right_vel"]
MIN_RANGE = -2.0
MAX_RANGE = 2.0
# motors's velocity should be between -2 and 2.


class TestHummingbot(unittest.TestCase):

    def __init__(self, *args):
        super(TestHummingbot, self).__init__(*args)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # node and subscribers are created.
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(SUB_TOPIC_NAMES[0], Float32, self.callback_left)
        rospy.Subscriber(SUB_TOPIC_NAMES[0], Float32, self.callback_right)

        time.sleep(1)
        # test methods should not be called until
        # callback gets the left and right speed.

    # test methods are runned by alphabetic sequence.

    def test_greater(self):
        """checks if the values are greater or equal to minimum number. """
        self.assertGreaterEqual(self.left_speed, MIN_RANGE)
        self.assertGreaterEqual(self.right_speed, MIN_RANGE)

    def test_less(self):
        """checks if the values are less or equal to maximum number. """
        self.assertLessEqual(self.left_speed, MAX_RANGE)
        self.assertLessEqual(self.right_speed, MAX_RANGE)

    def test_stop_service(self):
        """checks if the values are zero when stop service is called. """
        self.assertEqual(self.left_speed, 0)
        self.assertEqual(self.right_speed, 0)

    def callback_left(self, msg):
        self.left_speed = msg.data

    def callback_right(self, msg):
        self.right_speed = msg.data

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_hummingbot', TestHummingbot)
