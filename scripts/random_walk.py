#!/usr/bin/env python
"""
Random Walk Example Code

Written by: Zahi Kakish (zmk5)
License: BSD 3-Clause

"""
import argparse
import rospy
from geometry_msgs.msg import Twist
from pheeno_robot import PheenoRobot


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


def main():
    """
    Random Walk Pheeno

    In this demo task, the Pheeno robot undergoes 3 count turn in a
    specific direction followed by a 12 count linear motion. At 15
    count the direction, the count is reset and a new direction is
    chosen.

    """
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args.number == "":
        pheeno_number = ""

    else:
        pheeno_number = str(input_args.number)

    # Initialize Node
    rospy.init_node("pheeno_random_walk")

    # Create PheenoRobot Object
    pheeno = PheenoRobot(pheeno_number, 0.05, 0)

    # Other Important Variables
    count = 0
    cmd_vel_msg = Twist()
    linear_vel = 0.05
    angular_vel = 0.07

    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        if count == 0:
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = pheeno.random_turn() * angular_vel
            count += 1

        elif count == 3:
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = 0
            count += 1

        elif count == 15:
            count = 0

        else:
            count += 1

        pheeno.publish_cmd_vel(cmd_vel_msg)
        rate.sleep()



if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting 'pheeno_random_walk' node.")
