#!/usr/bin/env python

import rospy
import sys
import random
import argparse
from geometry_msgs.msg import Twist


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
    distribution. The value is then attributed to the 'var.angular.z'
    variable of geometry_msgs/Twist message.

    """
    if random.random() < 0.5:
        turn_direction = -0.05  # Turn left

    else:
        turn_direction = 0.05  # Turn right

    return turn_direction


if __name__ == "__main__":
    try:
        # Get arguments from argument parser.
        input_args = get_args()
        if input_args.number is "":
            pheeno_number = ""

        else:
            pheeno_number = "/pheeno_" + str(input_args.number)

        # Initialize Node
        rospy.init_node("pheeno_random_walk")

        # Craete Publishers
        pub = rospy.Publisher(pheeno_number + "/cmd_vel",
                              Twist,
                              queue_size=100)

        # Other Important Variables
        count = 0
        cmd_vel_msg = Twist()

        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if count is 0:
                cmd_vel_msg.linear.x = 0
                cmd__vel_msg.angular.z = random_turn()
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

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting 'pheeno_random_walk' node.")
