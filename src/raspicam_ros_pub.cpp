#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  // Variables
  raspicam::RaspiCam_Cv Camera;
  cv::Mat cv_image;
  char c;
  int delay = 30;  // 30ms delay
  const char *WIN_NAME = "Test Window";

  // Create ROS Node and Publisher
  ros::init(argc, argv, "raspicam_publisher");
  ros::NodeHandle node_obj;
  image_transpor::ImageTransport it(node_obj);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(30);

  // Create Message Variable
  sensor_msgs::ImagePtr msg;

  // Set camera parameters
  Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);   // Default  1280
  Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);  // Default   960
  // Camera.set(CV_CAP_PROP_BRIGHTNESS, 50);  // Default    50
  // Camera.set(CV_CAP_PROP_CONTRAST, 50);    // Default    50
  // Camera.set(CV_CAP_PROP_SATURATION, 50);  // Default    50
  // Camera.set(CV_CAP_PROP_GAIN, 50);        // Default    50
  // Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1); // Default CV_8UC1

  // Open Camera
  ROS_INFO("Opening Camera...");

  if (!Camera.open())
  {
    ROS_ERROR("Error opening the camera!");
  }

  while (node_obj.ok())
  {
    // Grab a new image
    Camera.grab();
    Camera.retrieve(cv_image);

    // Convert OpenCV image to ROS message and Publish Message
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    pub.publish(msg);

    // Exit the loop if 'ESC' key is pressed for longer than delay time.
    c = (char) cvWaitKey(delay);
    if (c == 27)
      break;

    // Spin and Sleep
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Release Pi Cam
  Camera.release();

  return 0;


}
