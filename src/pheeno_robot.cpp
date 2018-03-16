#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "pheeno_robot.h"
#include <vector>
#include <complex>
#include <cstdlib>
#include <iostream>

/*
 * Contructor for the PheenoRobot Class.
 *
 * The class constructor fills all std::vector variables with zeros
 * for use by the setters. Publishers and Subscribers are formally
 * defined as well.
 *
 */
PheenoRobot::PheenoRobot(std::string pheeno_name)
{
  ROS_INFO("Creating Pheeno Robot.");
  pheeno_namespace_id = pheeno_name;

  // Create IR sensor vector upon construction. (6 placements)
  for (int i = 0; i < 6; i++)
  {
    ir_sensor_values.push_back(0);
  }

  // Create Odom vector upon construction. (3 placements)
  for (int i = 0; i < 3; i++)
  {
    odom_pose_position.push_back(0);
    odom_twist_linear.push_back(0);
    odom_twist_angular.push_back(0);
  }

  // Create Odom vector upon construction. (4 placements)
  for (int i = 0; i < 4; i++)
  {
    odom_pose_orient.push_back(0);
  }

  // Instantiate the Publishers and Subscribers
  sub_ir_center_ = nh_.subscribe(pheeno_name + "/scan_center", 10,
                                 &PheenoRobot::irSensorCenterCallback, this);
  sub_ir_right_ = nh_.subscribe(pheeno_name + "/scan_right", 10,
                                &PheenoRobot::irSensorRightCallback, this);
  sub_ir_left_ = nh_.subscribe(pheeno_name + "/scan_left", 10,
                              &PheenoRobot::irSensorLeftCallback, this);
  sub_ir_cr_ = nh_.subscribe(pheeno_name + "/scan_cr", 10,
                             &PheenoRobot::irSensorCRightCallback, this);
  sub_ir_cl_ = nh_.subscribe(pheeno_name + "/scan_cl", 10,
                             &PheenoRobot::irSensorCLeftCallback, this);
  sub_ir_back_ = nh_.subscribe(pheeno_name + "/scan_back", 10,
                               &PheenoRobot::irSensorBackCallback, this);
  sub_ir_bottom_ = nh_.subscribe(pheeno_name + "/scan_bottom", 10,
                                 &PheenoRobot::irSensorBottomCallback, this);

  sub_odom_ = nh_.subscribe(pheeno_name + "/odom", 1,
                            &PheenoRobot::odomCallback, this);

  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(pheeno_name + "/cmd_vel", 100);

}

/*
 * Publishes command velocity (cmd_vel) messages.
 *
 * The specific Topic name that the message is published to is defined
 * in the Constructor for the PheenoRobot class.
 */
void PheenoRobot::publish(geometry_msgs::Twist velocity)
{
  pub_cmd_vel_.publish(velocity);
}

/*
 * Callback function for the IR Sensor (center) ROS subscriber.
 */
void PheenoRobot::irSensorCenterCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_center.data = msg->data;
  ir_sensor_values[0] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (back) ROS subscriber.
 */
void PheenoRobot::irSensorBackCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_back.data = msg->data;
  ir_sensor_values[1] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (right) ROS subscriber.
 */
void PheenoRobot::irSensorRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_right.data = msg->data;
  ir_sensor_values[2] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (left) ROS subscriber.
 */
void PheenoRobot::irSensorLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_left.data = msg->data;
  ir_sensor_values[3] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (center-right) ROS subscriber.
 */
void PheenoRobot::irSensorCRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_c_right.data = msg->data;
  ir_sensor_values[4] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (center-left) ROS subscriber.
 */
void PheenoRobot::irSensorCLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_c_left.data = msg->data;
  ir_sensor_values[5] = static_cast<double>(msg->data);
}

/*
 * Given a specific sensor limit, this member function returns a bool if
 * any of the sensors within ir_sensor_values is triggered.
 */
bool PheenoRobot::irSensorTriggered(float sensor_limit)
{
  int count = 0;
  for (int i = 0; i < 7; i++)
  {
    if (ir_sensor_values[i] < sensor_limit)
    {
      count += 1;
    }
  }

  // Greater than 1 instead of 0, because we aren't using the back IR sensor.
  return (count > 1) ? true : false;
}

/*
 * Callback function for the IR Sensor (bottom) ROS subscriber.
 *
 * Sets the data to either 0 or 1 if the message recieved is less than
 * 1600 or greater than 1600, respectively. The 1600 number is dependent
 * on the IR sensor used, therefore its value can be changed for use with
 * different applications.
 *
 * NOTE: ONLY FOR THE PHEENO MARKOV CHAIN EXPERIMENT.
 */
void PheenoRobot::irSensorBottomCallback(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data < 1600)
  {
    ir_sensor_bottom.data = 0;  // Black on the bottom
  }
  else
  {
    ir_sensor_bottom.data = 1;  // White on the bottom
  }
}

/*
 * Callback function for the odom ROS subscriber.
 *
 * This will set message data to odom pose (position and orientation) and twist
 * (linear and angular) variables.
 *
 * NOTE: ONLY USED IF `libgazebo_ros_p3d.so` PLUGIN IS IN THE XACRO FILE.
 */
void PheenoRobot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Assign values to appropriate pose information.
  odom_pose_position[0] = static_cast<double>(msg->pose.pose.position.x);
  odom_pose_position[1] = static_cast<double>(msg->pose.pose.position.y);
  odom_pose_position[2] = static_cast<double>(msg->pose.pose.position.z);
  odom_pose_orient[0] = static_cast<double>(msg->pose.pose.orientation.x);
  odom_pose_orient[1] = static_cast<double>(msg->pose.pose.orientation.y);
  odom_pose_orient[2] = static_cast<double>(msg->pose.pose.orientation.z);
  odom_pose_orient[3] = static_cast<double>(msg->pose.pose.orientation.w);

  // Assign values to appropriate twist information.
  odom_twist_linear[0] = static_cast<double>(msg->twist.twist.linear.x);
  odom_twist_linear[1] = static_cast<double>(msg->twist.twist.linear.y);
  odom_twist_linear[2] = static_cast<double>(msg->twist.twist.linear.z);
  odom_twist_angular[0] = static_cast<double>(msg->twist.twist.angular.x);
  odom_twist_angular[1] = static_cast<double>(msg->twist.twist.angular.y);
  odom_twist_angular[2] = static_cast<double>(msg->twist.twist.angular.z);
}

/*
 * Callback function for the Pi Cam ROS subscriber.
 *
 * Will recieve sensor_msgs.Image() messages and save only the current frame. If
 * the user needs to do image manipulation, use the CV_BRIDGE ros package to
 * convert the sensor_msgs.Image() data type to one useable by OpenCV.
 */
void PheenoRobot::piCamCallback()
{
  ROS_INFO("Not in use yet.");
}

/*
 * Generates a random turn direction (a sign change) based on a user provided value.
 */
double PheenoRobot::randomTurn(float angular)
{
  return rand() % 10 + 1 <= 5 ? (-1 * angular) : angular;
}

/*
 * Obstacle avoidance logic for a robot moving in a linear motion.
 *
 * Using class specific IR values, this simple if-else logic progresses by
 * comparing them to a certain range to avoid (default value). If triggered,
 * the references to linear and angular are set to specific values to make the
 * robot avoid the obstacle.
 */
void PheenoRobot::avoidObstaclesLinear(double& linear, double& angular, float angular_velocity, float linear_velocity, double range_to_avoid)
{
  if (ir_sensor_values[0] < range_to_avoid)
  {
    if (std::abs((ir_sensor_values[2] - ir_sensor_values[3])) < 5.0 ||
        (ir_sensor_values[2] > range_to_avoid && ir_sensor_values[3] > range_to_avoid))
    {
      linear = 0.0;
      angular = randomTurn();
    }

    if (ir_sensor_values[2] < ir_sensor_values[3])
    {
      linear = 0.0;
      angular = -1 * angular_velocity;  // Turn Left
    }
    else
    {
      linear = 0.0;
      angular = angular_velocity;  // Turn Right
    }
  }
  else if (ir_sensor_values[4] < range_to_avoid && ir_sensor_values[5] < range_to_avoid)
  {
    linear = 0.0;
    angular = randomTurn();
  }
  else if (ir_sensor_values[4] < range_to_avoid)
  {
    linear = 0.0;
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_values[5] < range_to_avoid)
  {
    linear = 0.0;
    angular = angular_velocity;  // Turn Right
  }
  else if (ir_sensor_values[2] < range_to_avoid)
  {
    linear = 0.0;
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_values[3] < range_to_avoid)
  {
    linear = 0.0;
    angular = angular_velocity;  // Turn Right
  }
  else
  {
    linear = linear_velocity;  // Move Straight
    angular = 0.0;
  }
}

/*
 * Obstacle avoidance logic for a robot moving in an angular (turning) motion.
 *
 * Using class specific IR values, this simple if-else logic progresses by
 * comparing them to a certain range to avoid (default value). If triggered,
 * the references to angular are set to specific values to make the robot avoid
 * the obstacle. Compared to the linear version of this callback member
 * function, this function only modifies the angular reference.
 */
void PheenoRobot::avoidObstaclesAngular(double& angular, double& random_turn_value, float angular_velocity, double range_to_avoid)
{
  if (ir_sensor_values[0] < range_to_avoid)
  {
    if (std::abs((ir_sensor_values[2] - ir_sensor_values[3])) < 5.0 ||
        (ir_sensor_values[2] > range_to_avoid && ir_sensor_values[3] > range_to_avoid))
    {
      angular = randomTurn();
    }

    if (ir_sensor_values[2] < ir_sensor_values[3])
    {
      angular = -1 * angular_velocity;  // Turn Left
    }
    else
    {
      angular = angular_velocity;  // Turn Right
    }
  }
  else if (ir_sensor_values[4] < range_to_avoid && ir_sensor_values[5] < range_to_avoid)
  {
    angular = randomTurn();
  }
  else if (ir_sensor_values[4] < range_to_avoid)
  {
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_values[5] < range_to_avoid)
  {
    angular = angular_velocity;  // Turn Right
  }
  else if (ir_sensor_values[2] < range_to_avoid)
  {
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_values[3] < range_to_avoid)
  {
    angular = angular_velocity;  // Turn Right
  }
  else
  {
    angular = random_turn_value;
  }

  if (angular != random_turn_value)
  {
    random_turn_value = angular;
  }
}
