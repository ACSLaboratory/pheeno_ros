#!/usr/bin/env python

import rospy
import sys
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


def print_callback(msg, sensor_location):
    """ Print encoder values in a easily legible fashion.

    Callback directed to from the ROS Subscribers. These are for printing
    IR sensor values for the pheeno.

    Parameters
    ----------
    msg : Float32
        Message recieved from topic '/scan_*'. center, back, right, left, cr,
        and cl specify IR Scanner locations. For exampe, '/scan_center' refers
        to the front facing IR sensor.
    sensor_location : str
        Location of IR sensors on the Pheeno. Will only come as one of six
        options (center, back, right, left, cr, cl).

    """
    global sensor_values
    sensor_values[sensor_location] = msg.data
    print("Center: %.2f\t Back: %.2f" %
          (sensor_values["center"], sensor_values["back"]))
    print("Right: %.2f\t Left: %.2f" %
          (sensor_values["right"], sensor_values["left"]))
    print("CR: %.2f\t CL: %.2f" %
          (sensor_values["cr"], sensor_values["cl"]))


if __name__ == "__main__":
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args.number is "":
        pheeno_number = ""

    else:
        pheeno_number = "/pheeno_" + str(input_args.number)

    # Initialize Node
    sys.stdout.write("Initializing 'read_sensor' node...")
    rospy.init_node("read_sensor")
    sys.stdout.write("done!\n")

    # Create Subscriber for encoder values.
    sub_scan_center = rospy.Subscriber(
        pheeno_number + "/scan_center", Float32, print_callback,
        callback_args="center")
    sub_scan_back = rospy.Subscriber(
        pheeno_number + "/scan_back", Float32, print_callback,
        callback_args="back")
    sub_scan_right = rospy.Subscriber(
        pheeno_number + "/scan_right", Float32, print_callback,
        callback_args="right")
    sub_scan_left = rospy.Subscriber(
        pheeno_number + "/scan_left", Float32, print_callback,
        callback_args="left")
    sub_scan_cr = rospy.Subscriber(
        pheeno_number + "/scan_cr", Float32, print_callback,
        callback_args="cr")
    sub_scan_cl = rospy.Subscriber(
        pheeno_number + "/scan_cl", Float32, print_callback,
        callback_args="cl")

    # Other variables
    global sensor_values
    sensor_values = {"center": 0.0, "back": 0.0, "right": 0.0, "left": 0.0,
                     "cr": 0.0, "cl": 0.0}

    rospy.spin()
