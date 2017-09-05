#include "ros/ros.h"
#include "sensor_listener.h"
#include "std_msgs/Float32.h"
#include <vector>

SensorListener::SensorListener()
{
  ROS_INFO("Creating Sensor Listener.");

  // Create IR sensor vector upon construction.
  for (int i = 0; i < 6; i++)
  {
    ir_sensor_values_.push_back(0);
  }
}

void SensorListener::irSensorCenterCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_center_.data = msg->data;
  ir_sensor_values_[0] = static_cast<double>(msg->data);
}

void SensorListener::irSensorBackCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_back_.data = msg->data;
  ir_sensor_values_[1] = static_cast<double>(msg->data);
}

void SensorListener::irSensorRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_right_.data = msg->data;
  ir_sensor_values_[2] = static_cast<double>(msg->data);
}

void SensorListener::irSensorLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_left_.data = msg->data;
  ir_sensor_values_[3] = static_cast<double>(msg->data);
}

void SensorListener::irSensorCRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_c_right_.data = msg->data;
  ir_sensor_values_[4] = static_cast<double>(msg->data);
}

void SensorListener::irSensorCLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_c_left_.data = msg->data;
  ir_sensor_values_[5] = static_cast<double>(msg->data);
}