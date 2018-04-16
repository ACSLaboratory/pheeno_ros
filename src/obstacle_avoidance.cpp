#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "command_line_parser.h"
#include "pheeno_robot.h"

int main(int argc, char **argv) {
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
  ros::Rate loop_rate(10);

  // Variables before loop
  double saved_time = ros::Time::now().toSec();
  double current_duration;
  double turn_direction = pheeno.randomTurn(0.07);
  double linear = 0.0;
  double angular = 0.0;
  geometry_msgs::Twist cmd_vel_msg;

  while (ros::ok())
  {
    // Find current duration of motion.
    current_duration = ros::Time::now().toSec() - saved_time;

    if (current_duration <= 2.0) {
      pheeno.avoidObstaclesLinear(linear, angular, turn_direction);
      cmd_vel_msg.linear.x = linear;
      cmd_vel_msg.angular.z = angular;

    } else if (current_duration < 5.0) {
      pheeno.avoidObstaclesAngular(angular, turn_direction);
      cmd_vel_msg.linear.x = linear;
      cmd_vel_msg.angular.z = angular;

    } else {
      // Reset Variables
      saved_time = ros::Time::now().toSec();
      turn_direction = pheeno.randomTurn(0.07);
    }

    // Publish, Spin, and Sleep
    pheeno.publish(cmd_vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
