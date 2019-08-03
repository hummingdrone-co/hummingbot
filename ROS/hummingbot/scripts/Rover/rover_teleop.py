#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Author : Mustafa Durmuş [mustafa@hummingdrone.co]

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
LIN_VEL_STEP_SIZE = 2.0
ANG_VEL_STEP_SIZE = 2.0

msg = """
Control Your Bot!
---------------------------
Moving around:
        w
   a    s    d
        x

space key, s : force stop

CTRL-C to quit
"""
e = """
Communications Failed
"""


def getKey():
    """
    reads the key user push and returns it.
    """
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

def print_current_vels(linear,angular):
      """
      prints current velocities.
      
      linear  : linear  velocity

      angular : angular velocity 
      """
      print("Current Linear Velocity:{} Current Angular Velocity:{}\n".format(linear,angular))

def publish_my_twist():
    """
    takes a key from user and publish a Twist message using step size.
    """
    print (msg)
    status = 0

    while(1):
        target_lin_vel = 0.0
        target_ang_vel = 0.0
        key = getKey()  # the key user push

        if key in ['w', 'x', 'a', 'd']:  # if key is a direction key (w,x,d,a)
            if key == 'w':
                target_lin_vel = LIN_VEL_STEP_SIZE
            elif key == 'x':
                target_lin_vel = -LIN_VEL_STEP_SIZE
            elif key == 'd':
                target_ang_vel = ANG_VEL_STEP_SIZE
            else:  # if key is 'a'
                target_ang_vel = -ANG_VEL_STEP_SIZE
            status = status + 1

        elif key == ' ' or key == 's':  # space or 's' stops the model.
            target_lin_vel = 0.0
            target_ang_vel = 0.0

        elif key == '\x03':
            break

        else:
            continue

        if status == 20:  # when user push 20 times, print control message again.
            print (msg)
            status = 0

        # Linearity: our model will only move to backward and forward, Y and Z axis will always be zero.
        twist.linear.x = target_lin_vel
        twist.linear.y = 0.0 ; twist.linear.z = 0.0
        # Angularity : Our model will only turn to right and left, X and Y axis will always be zero.
        twist.angular.x = 0.0 ; twist.angular.y = 0.0
        twist.angular.z = target_ang_vel
        pub.publish(twist)

        print_current_vels(target_lin_vel,target_ang_vel)



if __name__ == "__main__":
    if os.name != 'nt':  # if you are using Windows
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(PUB_TOPIC_NAME, Twist, queue_size=10)
    twist = Twist()

    try:
        publish_my_twist()
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
