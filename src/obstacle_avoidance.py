#!/usr/bin/env python

import rospy
import random
import argparse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def get_args():
    """ Get arguments from rosrun for individual deployment. """
    parser = argparse.ArgumentParser(
        description="Obstacle avoidance python script."
    )

    # Required arguments
    parser.add_argument("-n", "--number",
                        action="store",
                        required=False,
                        help="Add a pheeno number namespace.",
                        default="")

    # The rationale behind rospy.myargv()[1:] is provided here:
    # https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA
    return parser.parse_args(rospy.myargv()[1:])


def random_turn():
    """ Random turn direction.

    Returns a random turn direction for the pheeno based on a uniform
    distribution.

    """
    if random.random() < 0.5:
        turn_direction = -0.05  # Turn left

    else:
        turn_direction = 0.05  # Turn right

    return turn_direction


def obstacle_check(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits
    if msg.data < sensor_limits[ir_location]:
        sensor_triggered = ir_location
        is_obstacle_in_way[ir_location] = True

    else:
        is_obstacle_in_way[ir_location] = False


if __name__ == "__main__":
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args.number is "":
        pheeno_number = ""

    else:
        pheeno_number = "/pheeno_" + str(input_args.number)

    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Create Publishers and Subscribers
    pub = rospy.Publisher(
        pheeno_number + "/cmd_vel", Twist, queue_size=100)
    sub_ir_center = rospy.Subscriber(
        pheeno_number + "/scan_center", Float32, obstacle_check,
        callback_args="center")
    sub_ir_back = rospy.Subscriber(
        pheeno_number + "/scan_back", Float32, obstacle_check,
        callback_args="back")
    sub_ir_right = rospy.Subscriber(
        pheeno_number + "/scan_right", Float32, obstacle_check,
        callback_args="right")
    sub_ir_left = rospy.Subscriber(
        pheeno_number + "/scan_left", Float32, obstacle_check,
        callback_args="left")
    sub_ir_cr = rospy.Subscriber(
        pheeno_number + "/scan_cr", Float32, obstacle_check,
        callback_args="cr")
    sub_ir_cl = rospy.Subscriber(
        pheeno_number + "/scan_cl", Float32, obstacle_check,
        callback_args="cl")

    # Global Variables
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits

    # Other important variables
    is_obstacle_in_way = {"center": False, "cr": False, "right": False,
                          "back": False, "left": False, "cl": False}
    sensor_triggered = 0
    sensors = {"center": 0.05, "cr": -0.05, "right": -0.05,
               "back": -0.05, "left": 0.05, "cl": 0.05}
    sensor_limits = {"center": 25.0, "cr": 10.0, "right": 10.0,
                     "back": 15.0, "left": 10.0, "cl": 10.0}
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
            count = 3

        else:
            if count is 0:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = random_turn()
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 3:
                cmd_vel_msg.linear.x = 0.05
                cmd_vel_msg.angular.z = 0
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 15:
                count = 0

            else:
                pub.publish(cmd_vel_msg)
                count += 1

        rate.sleep()
