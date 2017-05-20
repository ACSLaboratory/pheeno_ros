#!/usr/bin/env python

import rospy
import argparse
import sys


def get_args():
    """ Get arguments from rosrun for individual deployment. """
    parser = argparse.ArgumentParser(
        description="Sets up package within the pheeno's directory."
    )

    # Required arguments
    parser.add_argument("-x", "--execute", action="execute", required=True,
                        help="something", default=False)

    # Optional arguments
    parser.add_argument("-s", "--save", action="store", required=False,
                        help="something", default=False)


if __name__ == "__main__":
    # Get arguments from command line.
    get_args()
