#!/usr/bin/env python

import rospy
import sys
import random
from geometry_msgs.msg import Twist


def random_turn():
    """ Random turn direction.

    Returns a random turn direction for the pheeno based on a uniform
    distribution. The value is then attributed to the 'var.angular.z'
    variable of geometry_msgs/Twist message.

    """
    if random.random() < 0.5:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -0.3  # Turn right

    else:
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.3  # Turn left

    return cmd_vel_msg


if __name__ == "__main__":
    # Initialize Node
    rospy.init_node("pheeno_random_walk")

    # Craete Publishers
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
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
