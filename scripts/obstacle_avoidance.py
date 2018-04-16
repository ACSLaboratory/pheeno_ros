#!/usr/bin/env python
"""
Obstacle Avoidance Example Code

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
    Obstacle Avoidance Pheeno

    In this demo task, the Pheeno robot does a random walk, but avoids
    obstacles that are a certain distance away from the Pheeno. Those
    sensor limits are shown in `sensor_limits`.

    """
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args.number == "":
        pheeno_number = ""

    else:
        pheeno_number = str(input_args.number)

    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Create PheenoRobot Object
    pheeno = PheenoRobot(pheeno_number, 0.05, 0)

    # Other important variables
    rate = rospy.Rate(10)
    saved_time = rospy.get_rostime().secs
    current_duration = 0
    linear_vel = 0.05
    angular_vel = 0.07
    cmd_vel_msg = Twist()
    ir_limit = 20

    while not rospy.is_shutdown():
        # Find current duration
        current_duration = rospy.get_rostime().secs - saved_time

        if current_duration <= 10:
            pheeno.avoid_obstacle(ir_limit, 0, angular_vel)
            cmd_vel_msg.linear.x = pheeno.linear_velocity
            cmd_vel_msg.angular.z = pheeno.angular_velocity

            # To avoid contradicting turns with obstacle avoidance, this
            # logic statement prevents the pheeno from going back and forth.
            if angular_vel != pheeno.angular_velocity:
                angular_vel = pheeno.angular_velocity

        elif current_duration < 20:
            pheeno.avoid_obstacle(ir_limit, linear_vel, 0)
            cmd_vel_msg.linear.x = pheeno.linear_velocity
            cmd_vel_msg.angular.z = pheeno.angular_velocity

        else:
            # Reset variables
            saved_time = rospy.get_rostime().secs
            angular_vel = pheeno.random_turn() * angular_vel

        # Publish to cmd_vel Topic and sleep
        pheeno.publish_cmd_vel(cmd_vel_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting 'pheeno_obstacle_avoidance' node.")
