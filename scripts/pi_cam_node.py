#!/usr/bin/env python

import rospy
import picamera
import picamera.array
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


def main():
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
            rate = rospy.Rate(2)

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
