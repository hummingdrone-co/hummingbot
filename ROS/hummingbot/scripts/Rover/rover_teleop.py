#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş [mustafa@hummingdrone.co]

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import os

if os.name == 'nt':  # means you work on Windows OS
    import msvcrt     # for windows terminal controls
else:
    import tty
    import termios  # for unix terminal controls

NODE_NAME = "teleop_node"
PUB_TOPIC_NAME = "/control"

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

HUMBOT_MAX_LIN_VEL = 3.0
HUMBOT_MAX_ANG_VEL = 2.0


msg = """
Control Your HummingBot!
---------------------------
Moving around:
        w
   a    s    d
        x

CTRL-C to quit
"""
e = """
Communications Failed
"""


def check_limits(input, low, high):
    """ 
    checks the limits of linearity and angularity.

    input : Desired velocity 
    low :  Minimum (Angular or Linear) velocity 
    high : Maximum (Angular or Linear) velocity
    """
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def get_key():
    """reads the pushed key and returns it."""
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_current_vels(linear, angular):
    """
    prints current velocities.

    linear  : linear  velocity

    angular : angular velocity
    """
    print("Current Linear Velocity:{} Current Angular Velocity:{}".format(
        linear, angular))


def publish_message():
    """
    takes a key from user and publish a Twist message using step size.
    """
    print (msg)
    status = 0

    while(1):
        key = get_key()

        # if the key is a direction key (w,x,d,a)
        if key in ['w', 'x', 'a', 'd']:
            if key == 'w':
                target_lin_vel = check_limits(
                    target_lin_vel+LIN_VEL_STEP_SIZE, -HUMBOT_MAX_LIN_VEL, HUMBOT_MAX_LIN_VEL)
            elif key == 'x':
                target_lin_vel = check_limits(
                    target_lin_vel-LIN_VEL_STEP_SIZE, -HUMBOT_MAX_LIN_VEL, HUMBOT_MAX_LIN_VEL)
            elif key == 'a':
                target_ang_vel = check_limits(
                    target_ang_vel+ANG_VEL_STEP_SIZE, -HUMBOT_MAX_ANG_VEL, HUMBOT_MAX_ANG_VEL)
            else:
                target_ang_vel = check_limits(
                    target_ang_vel-ANG_VEL_STEP_SIZE, -HUMBOT_MAX_ANG_VEL, HUMBOT_MAX_ANG_VEL)

            status = status + 1

#        elif key == ' ' or key == 's':  # space or 's' stops the model.
#            target_lin_vel = 0.0
#            target_ang_vel = 0.0

        elif key == '\x03':
            break

        elif key != '':
            print("Please use correct keys!")
            status = status + 1

        else:
            target_lin_vel = 0.0
            target_ang_vel = 0.0
#        else:
#            continue

        if status == 10:
            print (msg)
            status = 0

        twist.linear.x = target_lin_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = target_ang_vel
        pub.publish(twist)

        if target_ang_vel or target_lin_vel != 0.0:
            print_current_vels(target_lin_vel, target_ang_vel)


if __name__ == "__main__":
    if os.name != 'nt':  # for windows settings
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(PUB_TOPIC_NAME, Twist, queue_size=10)
    twist = Twist()

    try:
        publish_message()
    except:
        print (e)

    finally:  # make all message equal to zero at the end of the loop.

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
