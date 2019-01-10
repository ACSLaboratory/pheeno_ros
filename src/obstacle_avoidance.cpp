#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "pheeno_ros/command_line_parser.h"
#include "pheeno_ros/pheeno_robot.h"

int main(int argc, char **argv)
{
  // Initial Variables
  std::string pheeno_name;

  // Parse inputs
  CommandLineParser cml_parser(argc, argv);

  // Parse input arguments for Pheeno name.
  if (cml_parser["-n"])
  {
    std::string pheeno_number = cml_parser("-n");
    pheeno_name = "/pheeno_" + pheeno_number;
  }
  else
  {
    ROS_ERROR("Need to provide Pheeno number!");
  }

  // Initializing ROS node
  ros::init(argc, argv, "obstacle_avoidance_node");

  // Crate PheenoRobot Object
  PheenoRobot pheeno = PheenoRobot(pheeno_name);

  // ROS Rate loop
  ros::Rate loop_rate(15);

  // Variables before loop
  double linear = pheeno.getDefaultLinearVelocity();  // Initial vel is the default one provided by config file.
  double angular = 0.0;  // No turning.
  double max_range_to_avoid = 20.0;
  double min_range_to_avoid = 15.0;
  bool obs_flag = false;
  geometry_msgs::Twist cmd_vel_msg;

  while (ros::ok())
  {
    // Move and avoid obstacles if they are near.
    obs_flag = pheeno.avoidObstacleMove(linear, angular, max_range_to_avoid);
    obs_flag = pheeno.avoidObstacleStop(linear, angular, min_range_to_avoid);

    // Return values to default after obstacle avoided.
    if (!obs_flag)
    {
      linear = pheeno.getDefaultLinearVelocity();  // Default velocity
      angular = 0.0;
    }

    // Set values to cmd_vel msg
    cmd_vel_msg.linear.x = linear;
    cmd_vel_msg.angular.z = angular;

    // Publish, Spin, and Sleep
    pheeno.publishCmdVelocity(cmd_vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
