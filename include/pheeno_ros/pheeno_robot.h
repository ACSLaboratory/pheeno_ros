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

  // Default values for Pheeno.
  double range_to_avoid_;
  float linear_vel_;
  float angular_vel_;

  // Public Sensor Methods
  bool irSensorTriggered(float sensor_limits);

  // Public Movement Methods
  double randomTurn(float angular = 0.06);
  void avoidObstaclesLinear(double &linear, double &angular,
                            float angular_velocity = 1.2, float linear_velocity = 0.08,
                            double range_to_avoid = 20.0);
  void avoidObstaclesAngular(double &angular, double &random_turn_value,
                             float angular_velocity = 1.2, double range_to_avoid = 20.0);

  // Public Publishers
  void publish(geometry_msgs::Twist velocity);

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
