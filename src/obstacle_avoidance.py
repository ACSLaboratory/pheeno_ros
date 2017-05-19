#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def random_turn():
    """ Random turn direction.

    Returns a random turn direction for the pheeno based on a uniform
    distribution.

    """
    if random.random() < 0.5:
        turn_direction = -0.5  # Turn right

    else:
        turn_direction = 0.5  # Turn left

    return turn_direction


def obstacle_check(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    if msg.data < 15.0:
        sensor_triggered = ir_location
        is_obstacle_in_way[ir_location] = True

    else:
        is_obstacle_in_way[ir_location] = False


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
    is_obstacle_in_way = {"center": False, "cr": False, "right": False,
                          "back": False, "left": False, "cl": False}
    sensor_triggered = 0
    sensors = {"center": 0.5, "cr": -0.5, "right": -0.5,
               "back": -0.5, "left": 0.5, "cl": 0.5}
    cmd_vel_msg = Twist()
    obs_cmd_vel_msg = Twist()
    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        if True in is_obstacle_in_way.values():
            obs_cmd_vel_msg.linear.x = 0
            obs_cmd_vel_msg.angular.z = sensors[sensor_triggered]
            pub.publish(obs_cmd_vel_msg)

            # Prevent a random turn into an object after trying to avoid.
            count = 10

        else:
            if count is 0:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = random_turn()
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 3:
                cmd_vel_msg.linear.x = 0.07
                cmd_vel_msg.angular.z = 0
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 15:
                count = 0

            else:
                pub.publish(cmd_vel_msg)
                count += 1

        rate.sleep()
