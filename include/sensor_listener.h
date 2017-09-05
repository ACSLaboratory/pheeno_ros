#ifndef PROJECT_SENSOR_LISTENER_H
#define PROJECT_SENSOR_LISTENER_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <vector>

class SensorListener {

public:
  // Sensor Messages
  std_msgs::Float32 ir_sensor_center_;
  std_msgs::Float32 ir_sensor_back_;
  std_msgs::Float32 ir_sensor_right_;
  std_msgs::Float32 ir_sensor_left_;
  std_msgs::Float32 ir_sensor_c_right_;
  std_msgs::Float32 ir_sensor_c_left_;
  std::vector<double> ir_sensor_values_;

  // Constructor
  SensorListener();

  // Modules
  void irSensorCenterCallback(const std_msgs::Float32::ConstPtr& msg);
  void irSensorBackCallback(const std_msgs::Float32::ConstPtr& msg);
  void irSensorRightCallback(const std_msgs::Float32::ConstPtr& msg);
  void irSensorLeftCallback(const std_msgs::Float32::ConstPtr& msg);
  void irSensorCRightCallback(const std_msgs::Float32::ConstPtr& msg);
  void irSensorCLeftCallback(const std_msgs::Float32::ConstPtr& msg);

};

#endif //PROJECT_SENSOR_LISTENER_H
