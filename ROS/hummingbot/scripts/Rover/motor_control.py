#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş [mustafa@hummingdrone.co]

import rospy
from std_msgs.msg import Float32

NODE_NAME = "motor_control"
SUB_TOPIC_NAMES = ["/gazebo_hummingbot/left_vel",
                   "/gazebo_hummingbot/right_vel"]


def set_motor_velocity(speed, direction):
    """
    Takes a float number and sets motor velocity of any wheel.

    speed   : Velocity of the wheel that taken from joystick

    direction : Wheel name (left or right)

    """
    if direction == "left":
        pass
    else:
        pass


def callback_left(vel):
    set_motor_velocity(vel, direction="left")


def callback_right(vel):
    set_motor_velocity(vel, direction="right")

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    
    sub_left  = rospy.Subscriber(SUB_TOPIC_NAMES[0], Float32, callback_left)
    sub_right = rospy.Subscriber(SUB_TOPIC_NAMES[1], Float32, callback_right)
    rospy.spin()
