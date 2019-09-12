#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author : Mustafa Durmuş [mustafa@hummingdrone.co]

import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String, Float32

NODE_NAME = "motor_control"
SUB_TOPIC_NAMES = ["/gazebo_hummingbot/left_vel",
                   "/gazebo_hummingbot/right_vel"]


# stops all motors
def all_stop():

    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)


def set_motor_speed(speed, motor_id):
    """
    Takes a float number and sets motor speed of any wheel.

    speed   : Velocity of the wheel that taken from joystick

    motor_id : ID of left or right wheel

    """
    if motor_id == 1:
        motor = motor_left
    else:
        motor = motor_right

    motor.setSpeed(int(abs(speed)))

    if speed >= 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)


def callback_left(velocity):
    set_motor_speed(velocity.data, motor_id=1)


def callback_right(velocity):
    set_motor_speed(velocity.data, motor_id=2)


# initialization
if __name__ == '__main__':

    # setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    # initialize id for left and right motor
    motor_left_ID  = 1
    motor_right_ID = 2

    # get motor drivers
    motor_left =  motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)

    # stop the motors as precaution
    all_stop()

    # setup ros node
    rospy.init_node(NODE_NAME)

    rospy.Subscriber(SUB_TOPIC_NAMES[0], Float32, callback_left)
    rospy.Subscriber(SUB_TOPIC_NAMES[1], Float32, callback_right)

    # start running
    rospy.spin()

    # stop motors before exiting
    all_stop()
