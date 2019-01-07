#ifndef PHEENO_ROS_PHEENO_ROBOT_H
#define PHEENO_ROS_PHEENO_ROBOT_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>

namespace Pheeno
{
  enum IR
  {
    CENTER,
    BACK,
    RIGHT,
    LEFT,
    CRIGHT,
    CLEFT
  };

  enum ENCODER
  {
    LL,
    LR,
    RL,
    RR
  };
}

class PheenoRobot
{

public:
  // Constructor
  PheenoRobot(std::string pheeno_name);

  // Pheeno name
  std::string pheeno_namespace_id_;

  // Sensor Messages
  std::vector<double> ir_sensor_vals_;
  std::vector<int> encoder_vals_;
  std::vector<double> magnetometer_vals_;
  std::vector<double> gyroscope_vals_;
  std::vector<double> accelerometer_vals_;

  // Odometry Messages
  nav_msgs::Odometry odom_msg_;
  std::vector<double> odom_pose_position_;
  std::vector<double> odom_pose_orient_;
  std::vector<double> odom_twist_linear_;
  std::vector<double> odom_twist_angular_;

  // Camera Messages
  std::vector<bool> color_state_facing_;

  // Public Sensor Methods
  bool irSensorTriggered(float sensor_limits);

  // Public Movement Methods
  double randomTurn(double angular);
  double getLinearVelocity();
  double getAngularVelocity();
  void setLinearVelocity(double new_linear_velocity);
  void setAngularVelocity(double new_angular_velocity);
  void avoidObstacleMove(double& linear, double& angular, double range_to_avoid);
  void avoidObstacleStop(double& linear, double& angular, double range_to_avoid);

  // Public Publishers
  void publishCmdVelocity(geometry_msgs::Twist velocity);

  // Public camera methods
  bool checkFrontColor(int color);

private:
  // ROS Node handle
  ros::NodeHandle nh_;

  // Private Subscribers
  ros::Subscriber sub_ir_center_;
  ros::Subscriber sub_ir_right_;
  ros::Subscriber sub_ir_left_;
  ros::Subscriber sub_ir_cr_;
  ros::Subscriber sub_ir_cl_;
  ros::Subscriber sub_ir_back_;
  ros::Subscriber sub_ir_bottom_;
  ros::Subscriber sub_pheeno_cam_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_encoder_LL_;
  ros::Subscriber sub_encoder_LR_;
  ros::Subscriber sub_encoder_RL_;
  ros::Subscriber sub_encoder_RR_;
  ros::Subscriber sub_magnetometer_;
  ros::Subscriber sub_gyroscope_;
  ros::Subscriber sub_accelerometer_;

  // Private Publishers
  ros::Publisher pub_cmd_vel_;

  // Default values for Pheeno.
  double max_range_to_avoid_;
  double min_range_to_avoid_;
  double linear_vel_;
  double angular_vel_;
  double def_linear_vel_;
  double def_angular_vel_;
  double obs_linear_vel_;
  double obs_angular_vel_;

  // IR Callback Methods
  void irSensorCallback(const std_msgs::Float32::ConstPtr& msg, int ir_location);

  // Odom Callback Methods
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  // Encoder Callback Methods
  void encoderCallback(const std_msgs::Int16::ConstPtr& msg, int encoder_location);

  // Other Sensor Callback Methods
  void magnetometerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void gyroscopeCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void accelerometerCallback(const geometry_msgs::Vector3::ConstPtr& msg);

  // Camera Callback Modules
  void piCamCallback();
};

#endif // PHEENO_ROS_PHEENO_ROBOT_H
