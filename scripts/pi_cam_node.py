#!/usr/bin/env python
"""
Pi Camera ROS Node

Written by: Zahi Kakish (zmk5)
License: BSD 3-Clause

"""
import sys
import rospy
import picamera
import picamera.array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    """
    Sample Pi Camera ROS Node

    This simple ROS node script takes images from a Pi Camera attached to a
    Pheeno's Raspberry Pi (or other Robot that uses the Pi Camera) and
    Publishes the images as a ROS topic.

    NOTE: This method of publishing images is very computationally intensive,
    especially on a Raspberry Pi. Use when needed or try to do image processing
    within a node and only publish the results. For example, checking if a
    color is located within an image and then publishing True or False instead
    of the image.

    """
    # Select camera for use.
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            # Initialize Node
            sys.stdout.write("Initializing 'pi_cam' node...")
            rospy.init_node("pi_cam")
            sys.stdout.write("done!\n")

            # Create Publisher to publish pi_cam images.
            sys.stdout.write("Creating Publisher...")
            pub = rospy.Publisher("/pi_cam/image_raw",
                                  Image,
                                  queue_size=307200)
            sys.stdout.write("done!\n")

            # Other important variables
            camera.resolution = (640, 480)
            bridge = CvBridge()
            rate = rospy.Rate(10)

            # Starting node
            sys.stdout.write("Starting node!\n")
            while not rospy.is_shutdown():
                try:
                    # Capture and publish
                    camera.capture(stream, 'bgr', use_video_port=True)
                    image_message = bridge.cv2_to_imgmsg(
                        stream.array, encoding="bgr8")
                    pub.publish(image_message)

                    # Reset stream for next capture
                    stream.truncate(0)
                    rate.sleep()

                except KeyboardInterrupt:
                    sys.exit("Closing node!\n")



if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting 'pi_cam' node.")
