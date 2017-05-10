#!/usr/bin/env python

import rospy
import sys
import std_msgs.msg import Int16


def print_callback(msg):
    """ Print encoder values in a easily legible fashion.

    Callback directed to from the ROS Subscribers. These are for printing
    encoder values for the pheeno.

    Parameters
    ----------
    msg : Int16
        Message recieved from topic '/encoder_*'. LL, LR, RL, and RR specify
        the H-bridge location (Left or Right) and the motor location (Left or
        Right). For exampe, '/encoder_LL' refers to the Left H-bridge and the
        Left motor, respectively.

    """
    msg.data


if __name__ == "__main__":
    # Initialize Node
    sys.stdout.write("Initializing 'read_encoder' node...")
    rospy.init_node("read_encoder")
    sys.stdout.write("done!\n")

    # Create Subscriber for encoder values.
    sub_encoder_LL = rospy.Subscriber("/encoder_LL", Int16, print_callback)
    sub_encoder_LR = rospy.Subscriber("/encoder_LR", Int16, print_callback)
    sub_encoder_RL = rospy.Subscriber("/encoder_RL", Int16, print_callback)
    sub_encoder_RR = rospy.Subscriber("/encoder_RR", Int16, print_callback)

    rospy.spin()
