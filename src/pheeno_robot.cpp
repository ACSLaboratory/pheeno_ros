#include "pheeno_ros/pheeno_robot.h"

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
  pheeno_namespace_id_ = pheeno_name;

  // Create IR sensor vector upon construction. (6 placements)
  for (int i = 0; i < 6; i++)
  {
    ir_sensor_vals_.push_back(0);
  }

  // Create sensor vector upon construction. (3 placements)
  for (int i = 0; i < 3; i++)
  {
    odom_pose_position_.push_back(0);
    odom_twist_linear_.push_back(0);
    odom_twist_angular_.push_back(0);

    magnetometer_vals_.push_back(0);
    gyroscope_vals_.push_back(0);
    accelerometer_vals_.push_back(0);
  }

  // Create sensor vector upon construction. (4 placements)
  for (int i = 0; i < 4; i++)
  {
    odom_pose_orient_.push_back(0);
    encoder_vals_.push_back(0);
  }

  // Use rosparams to fill defaults.
  nh_.getParam("/pheeno_robot/range_to_avoid", range_to_avoid_);
  nh_.getParam("/pheeno_robot/linear_velocity", linear_vel_);
  nh_.getParam("/pheeno_robot/angular_velocity", angular_vel_);

  // IR Sensor Subscribers
  sub_ir_center_ = nh_.subscribe(pheeno_name + "/scan_center", 10, &PheenoRobot::irSensorCenterCallback, this);
  sub_ir_right_ = nh_.subscribe(pheeno_name + "/scan_right", 10, &PheenoRobot::irSensorRightCallback, this);
  sub_ir_left_ = nh_.subscribe(pheeno_name + "/scan_left", 10, &PheenoRobot::irSensorLeftCallback, this);
  sub_ir_cr_ = nh_.subscribe(pheeno_name + "/scan_cr", 10, &PheenoRobot::irSensorCRightCallback, this);
  sub_ir_cl_ = nh_.subscribe(pheeno_name + "/scan_cl", 10, &PheenoRobot::irSensorCLeftCallback, this);
  sub_ir_back_ = nh_.subscribe(pheeno_name + "/scan_back", 10, &PheenoRobot::irSensorBackCallback, this);

  // Odom Subscriber
  sub_odom_ = nh_.subscribe(pheeno_name + "/odom", 1, &PheenoRobot::odomCallback, this);

  // Encoder Subscribers
  sub_encoder_LL_ = nh_.subscribe(pheeno_name + "/encoder_LL", 10, &PheenoRobot::encoderLLCallback, this);
  sub_encoder_LR_ = nh_.subscribe(pheeno_name + "/encoder_LR", 10, &PheenoRobot::encoderLRCallback, this);
  sub_encoder_RL_ = nh_.subscribe(pheeno_name + "/encoder_RL", 10, &PheenoRobot::encoderRLCallback, this);
  sub_encoder_RR_ = nh_.subscribe(pheeno_name + "/encoder_RR", 10, &PheenoRobot::encoderRRCallback, this);

  // Magnetometer, Gyroscope, Accelerometer Subscriber
  sub_magnetometer_ = nh_.subscribe(pheeno_name + "/magnetometer", 10, &PheenoRobot::magnetometerCallback, this);
  sub_gyroscope_ = nh_.subscribe(pheeno_name + "/gyroscope", 10, &PheenoRobot::gyroscopeCallback, this);
  sub_accelerometer_ = nh_.subscribe(pheeno_name + "/accelerometer", 10, &PheenoRobot::accelerometerCallback, this);

  // cmd_vel Publisher
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
  ir_sensor_vals_[Pheeno::IR::CENTER] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (back) ROS subscriber.
 */
void PheenoRobot::irSensorBackCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_vals_[Pheeno::IR::BACK] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (right) ROS subscriber.
 */
void PheenoRobot::irSensorRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_vals_[Pheeno::IR::RIGHT] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (left) ROS subscriber.
 */
void PheenoRobot::irSensorLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_vals_[Pheeno::IR::LEFT] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (center-right) ROS subscriber.
 */
void PheenoRobot::irSensorCRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_vals_[Pheeno::IR::CRIGHT] = static_cast<double>(msg->data);
}

/*
 * Callback function for the IR Sensor (center-left) ROS subscriber.
 */
void PheenoRobot::irSensorCLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ir_sensor_vals_[Pheeno::IR::CLEFT] = static_cast<double>(msg->data);
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
    if (ir_sensor_vals_[i] < sensor_limit)
    {
      count += 1;
    }
  }

  // Greater than 1 instead of 0, because we aren't using the back IR sensor.
  return (count > 1) ? true : false;
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
  odom_pose_position_[0] = static_cast<double>(msg->pose.pose.position.x);
  odom_pose_position_[1] = static_cast<double>(msg->pose.pose.position.y);
  odom_pose_position_[2] = static_cast<double>(msg->pose.pose.position.z);
  odom_pose_orient_[0] = static_cast<double>(msg->pose.pose.orientation.x);
  odom_pose_orient_[1] = static_cast<double>(msg->pose.pose.orientation.y);
  odom_pose_orient_[2] = static_cast<double>(msg->pose.pose.orientation.z);
  odom_pose_orient_[3] = static_cast<double>(msg->pose.pose.orientation.w);

  // Assign values to appropriate twist information.
  odom_twist_linear_[0] = static_cast<double>(msg->twist.twist.linear.x);
  odom_twist_linear_[1] = static_cast<double>(msg->twist.twist.linear.y);
  odom_twist_linear_[2] = static_cast<double>(msg->twist.twist.linear.z);
  odom_twist_angular_[0] = static_cast<double>(msg->twist.twist.angular.x);
  odom_twist_angular_[1] = static_cast<double>(msg->twist.twist.angular.y);
  odom_twist_angular_[2] = static_cast<double>(msg->twist.twist.angular.z);
}

/*
 * Callback function for the Encoder LL ROS subscriber.
 */
void PheenoRobot::encoderLLCallback(const std_msgs::Int16::ConstPtr& msg)
{
  encoder_vals_[Pheeno::ENCODER::LL] = static_cast<int>(msg->data);
}

/*
 * Callback function for the Encoder LR ROS subscriber.
 */
void PheenoRobot::encoderLRCallback(const std_msgs::Int16::ConstPtr& msg)
{
  encoder_vals_[Pheeno::ENCODER::LR] = static_cast<int>(msg->data);
}

/*
 * Callback function for the Encoder RL ROS subscriber.
 */
void PheenoRobot::encoderRLCallback(const std_msgs::Int16::ConstPtr& msg)
{
  encoder_vals_[Pheeno::ENCODER::RL] = static_cast<int>(msg->data);
}

/*
 * Callback function for the Encoder RR ROS subscriber.
 */
void PheenoRobot::encoderRRCallback(const std_msgs::Int16::ConstPtr& msg)
{
  encoder_vals_[Pheeno::ENCODER::RR] = static_cast<int>(msg->data);
}

/*
 * Callback function for the Magnetometer sensor ROS subscriber.
 */
void PheenoRobot::magnetometerCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  magnetometer_vals_[0] = static_cast<double>(msg->x);
  magnetometer_vals_[1] = static_cast<double>(msg->y);
  magnetometer_vals_[2] = static_cast<double>(msg->z);
}

/*
 * Callback function for the Gyroscope sensor ROS subscriber.
 */
void PheenoRobot::gyroscopeCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  gyroscope_vals_[0] = static_cast<double>(msg->x);
  gyroscope_vals_[1] = static_cast<double>(msg->y);
  gyroscope_vals_[2] = static_cast<double>(msg->z);
}

/*
 * Callback function for the Accelerometer sensor ROS subscriber.
 */
void PheenoRobot::accelerometerCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  accelerometer_vals_[0] = static_cast<double>(msg->x);
  accelerometer_vals_[1] = static_cast<double>(msg->y);
  accelerometer_vals_[2] = static_cast<double>(msg->z);
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
void PheenoRobot::avoidObstaclesLinear(double& linear, double& angular, float angular_velocity, float linear_velocity,
                                       double range_to_avoid)
{
  if (ir_sensor_vals_[Pheeno::IR::CENTER] < range_to_avoid)
  {
    if (std::abs((ir_sensor_vals_[Pheeno::IR::RIGHT] - ir_sensor_vals_[Pheeno::IR::LEFT])) < 5.0 ||
        (ir_sensor_vals_[Pheeno::IR::RIGHT] > range_to_avoid && ir_sensor_vals_[Pheeno::IR::LEFT] > range_to_avoid))
    {
      linear = 0.0;
      angular = randomTurn();
    }

    if (ir_sensor_vals_[Pheeno::IR::RIGHT] < ir_sensor_vals_[Pheeno::IR::LEFT])
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
  else if (ir_sensor_vals_[Pheeno::IR::CRIGHT] < range_to_avoid && ir_sensor_vals_[Pheeno::IR::CLEFT] < range_to_avoid)
  {
    linear = 0.0;
    angular = randomTurn();
  }
  else if (ir_sensor_vals_[Pheeno::IR::CRIGHT] < range_to_avoid)
  {
    linear = 0.0;
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_vals_[Pheeno::IR::CLEFT] < range_to_avoid)
  {
    linear = 0.0;
    angular = angular_velocity;  // Turn Right
  }
  else if (ir_sensor_vals_[Pheeno::IR::RIGHT] < range_to_avoid)
  {
    linear = 0.0;
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_vals_[Pheeno::IR::LEFT] < range_to_avoid)
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
void PheenoRobot::avoidObstaclesAngular(double& angular, double& random_turn_value, float angular_velocity,
                                        double range_to_avoid)
{
  if (ir_sensor_vals_[Pheeno::IR::CENTER] < range_to_avoid)
  {
    if (std::abs((ir_sensor_vals_[Pheeno::IR::RIGHT] - ir_sensor_vals_[Pheeno::IR::LEFT])) < 5.0 ||
        (ir_sensor_vals_[Pheeno::IR::RIGHT] > range_to_avoid && ir_sensor_vals_[Pheeno::IR::LEFT] > range_to_avoid))
    {
      angular = randomTurn();
    }

    if (ir_sensor_vals_[Pheeno::IR::RIGHT] < ir_sensor_vals_[Pheeno::IR::LEFT])
    {
      angular = -1 * angular_velocity;  // Turn Left
    }
    else
    {
      angular = angular_velocity;  // Turn Right
    }
  }
  else if (ir_sensor_vals_[Pheeno::IR::CRIGHT] < range_to_avoid && ir_sensor_vals_[Pheeno::IR::CLEFT] < range_to_avoid)
  {
    angular = randomTurn();
  }
  else if (ir_sensor_vals_[Pheeno::IR::CRIGHT] < range_to_avoid)
  {
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_vals_[Pheeno::IR::CLEFT] < range_to_avoid)
  {
    angular = angular_velocity;  // Turn Right
  }
  else if (ir_sensor_vals_[Pheeno::IR::RIGHT] < range_to_avoid)
  {
    angular = -1 * angular_velocity;  // Turn Left
  }
  else if (ir_sensor_vals_[Pheeno::IR::LEFT] < range_to_avoid)
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
