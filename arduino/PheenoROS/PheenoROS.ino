#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "PheenoV2Basic.h"

// Create ROS Node instance
ros::NodeHandle nh;

// Create Pheeno Instance
PheenoV2Basic pheeno_robot = PheenoV2Basic(2);

// Create Messages for Publishers
std_msgs::Float32 scan_center_msg;
std_msgs::Float32 scan_back_msg;
std_msgs::Float32 scan_right_msg;
std_msgs::Float32 scan_left_msg;
std_msgs::Float32 scan_cr_msg;
std_msgs::Float32 scan_cl_msg;
std_msgs::Int16 encoder_LL_msg;
std_msgs::Int16 encoder_LR_msg;
std_msgs::Int16 encoder_RL_msg;
std_msgs::Int16 encoder_RR_msg;
// std_msgs::Float32 mag_msg;
// std_msgs::Float32 gyro_msg;
// std_msgs::Float32 accel_msg;

// Create Publishers
ros::Publisher pub_ir_center("/scan_center");  // Center IR Sensor
ros::Publisher pub_ir_back("/scan_back");      // Back IR Sensor
ros::Publisher pub_ir_right("/scan_right");    // Right IR Sensor
ros::Publisher pub_ir_left("/scan_left");      // Left IR Sensor
ros::Publisher pub_ir_cr("/scan_cr");          // Center Right IR Sensor
ros::Publisher pub_ir_cl("/scan_cl");          // Center Left IR Sensor
ros::Publisher pub_encoder_LL("/encoder_LL");  // Encoder LL
ros::Publisher pub_encoder_LR("/encoder_LR");  // Encoder LR
ros::Publisher pub_encoder_RL("/encoder_RL");  // Encoder RL
ros::Publisher pub_encoder_RR("/encoder_RR");  // Encoder RR
// ros::Publisher pub_mag("/mag");                // Magnetometer
// ros::Publisher pub_gyro("/gyro");              // Gyroscope
// ros::Publisher pub_accel("/accel");            // Accelerometer

// Callback for Subscribers
void callback(const geometry_msgs::Twist &msg) {
  if (msg.linear.x > 0) {
    PheenoMoveForward();
  } else if (msg.linear.x < 0) {
    PheenoMoveReverse();
  }
  
  if (msg.angular.z > 0) {
    PheenoTurnRight();
  } else if (msg.angular.z < 0) {
    PheenoTurnLeft();
  }

}

// Create Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", callback);

void setup() {
  // Setup Pheeno robot
  pheeno_robot.SetupBasic();

  // Initialize ROS Node
  nh.initNode();

  // Start Advertising
  nh.advertise(pub_ir_center);
  nh.advertise(pub_ir_back);
  nh.advertise(pub_ir_right);
  nh.advertise(pub_ir_left);
  nh.advertise(pub_ir_cr);
  nh.advertise(pub_ir_cl);
  nh.advertise(pub_encoder_LL);
  nh.advertise(pub_encoder_LR);
  nh.advertise(pub_encoder_RL);
  nh.advertise(pub_encoder_RR);
  // nh.advertise(pub_mag);
  // nh.advertise(pub_gyro);
  // nh.advertise(pub_accel);

  // Start Subscribing
  nh.subscribe(sub_cmd_vel);

}


void loop() {

  // Refresh Sensor Readings
  pheeno_robot.readIR();
  pheeno_robot.readEncoders();
  // pheeno_robot.readMag();
  // pheeno_robot.readGyro();
  // pheeno_robot.readAccel();
  
  // Assign sensor values to Msg variable
  scan_center_msg = pheeno_robot.CDistance;
  scan_back_msg = pheeno_robot.BDistance;
  scan_right_msg = pheeno_robot.RDistance;
  scan_left_msg = pheeno_robot.LDistance;
  scan_cr_msg = pheeno_robot.RFDistance;
  scan_cl_msg = pheeno_robot.LFDistance;
  encoder_LL_msg = pheeno_robot.encoderCountLL;
  encoder_LR_msg = pheeno_robot.encoderCountLR;
  encoder_RL_msg = pheeno_robot.encoderCountRL;
  encoder_RR_msg = pheeno_robot.encoderCountRR;
  
  // Publish the Topics
  pub_ir_center.publish(&scan_center_msg);
  pub_ir_back.publish(&scan_back_msg);
  pub_ir_right.publish(&scan_right_msg); 
  pub_ir_left.publish(&scan_left_msg);
  pub_ir_cr.publish(&scan_cr_msg);
  pub_ir_cl.publish(&scan_cl_msg);
  pub_encoder_LL.publish(&encoder_LL_msg);
  pub_encoder_LR.publish(&encoder_LR_msg);
  pub_encoder_RL.publish(&encoder_RL_msg);
  pub_encoder_RR.publish(&encoder_RR_msg);

  nh.spinOnce();
  
}


// Turns the Pheeno left.
// Currently the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnLeft(int speed) {
  my_robot.reverseRL(speed);
  my_robot.forwardLR(speed);

}


// Turns the Pheeno Right.
// Currently, the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnRight(int speed) {
  my_robot.reverseLR(speed);
  my_robot.forwardRL(speed);

}


// Moves the Pheeno Forward.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveForward(int speed) {
  my_robot.forwardLR(speed);
  my_robot.forwardRL(speed);

}


// Moves the Pheeno Reverse.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveReverse(int speed) {
  my_robot.reverseLR(speed);
  my_robot.reverseRL(speed);

}
