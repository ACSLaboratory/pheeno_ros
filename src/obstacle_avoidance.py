#!/usr/bin/env python

import rospy
import random
import geometry_msgs.msg import Twist
import std_msgs.msg import Float32


def random_turn():
    """ Random turn direction.

    Returns a random turn direction for the pheeno based on a uniform
    distribution. The value is then attributed to the 'var.angular.z'
    variable of geometry_msgs/Twist message.

    """
    cmd_vel_msg = Twist()
    if random.random() < 0.5:
        cmd_vel_msg.angular.z = -0.3  # Turn right

    else:
        cmd_vel_msg.angular.z = 0.3  # Turn left

    return cmd_vel_msg


def obstacle_check(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    if msg.data is not 0:
        sensor_triggered = ir_location
        is_obstacle_in_way = True


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Create Publishers and Subscribers
    pub = rospy.Publisher(
        "/cmd_vel", Twist, queue_size=100)
    sub_ir_center = rospy.Subscriber(
        "/scan_center", Float32, obstacle_check, callback_args="center")
    sub_ir_back = rospy.Subscriber(
        "/scan_back", Float32, obstacle_check, callback_args="back")
    sub_ir_right = rospy.Subscriber(
        "/scan_right", Float32, obstacle_check, callback_args="right")
    sub_ir_left = rospy.Subscriber(
        "/scan_left", Float32, obstacle_check, callback_args="left")
    sub_ir_cr = rospy.Subscriber(
        "/scan_cr", Float32, obstacle_check, callback_args="cr")
    sub_ir_cl = rospy.Subscriber(
        "/scan_cl", Float32, obstacle_check, callback_args="cl")

    # Global Variables
    global is_obstacle_in_way
    global sensor_triggered

    # Other important variables
    is_obstacle_in_way = False
    sensor_triggered = 0
    sensors = {"center": 0.3, "cr": 0.3, "right": 0.3,
               "back": -0.3, "left": -0.3, "cl": -0.3}
    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        if is_obstacle_in_way:
            ang_vel_msg = Twist()
            ang_vel_msg.angular.z = 0.3
            pub.publish(ang_vel_msg)

        else:
            if count is 0:
                ang_vel_msg = random_turn()
                pub.publish(ang_vel_msg)
                count += 1

            elif 1 <= count <= 15:
                pub.publish(ang_vel_msg)
                count += 1

            elif 15 < count < 35:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.07
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 35:
                count = 0

        rate.sleep()
