#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "command_line_parser.h"
#include "pheeno_robot.h"

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
    std::cout << pheeno_name << std::endl;
  }
  else
  {
    ROS_ERROR("Need to provide Pheeno number!");
  }

  // Initializing ROS node
  ros::init(argc, argv, "random_walk_node");

  // Create PheenoRobot object
  PheenoRobot pheeno = PheenoRobot(pheeno_name);

  // ROS Rate loop
  ros::Rate loop_rate(10);

  // Variables before loop
  double saved_time = ros::Time::now().toSec();
  double current_duration;
  double angular = 0.07;
  double turn_direction = pheeno.randomTurn(angular);
  geometry_msgs::Twist cmd_vel_msg;

  while (ros::ok())
  {
    // Find current duration of motion.
    current_duration = ros::Time::now().toSec() - saved_time;

    if (current_duration <= 10.0) {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = turn_direction;

    } else if (current_duration < 20.0) {
      cmd_vel_msg.linear.x = 0.05;
      cmd_vel_msg.angular.z = 0.0;

    } else {
      // Reset Variables
      saved_time = ros::Time::now().toSec();
      turn_direction = pheeno.randomTurn(angular);
    }

    // Publish, Spin, and Sleep
    pheeno.publish(cmd_vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
