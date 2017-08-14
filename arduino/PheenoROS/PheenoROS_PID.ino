#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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
std_msgs::Int16 encoder_LL_msg;  // Left HBridge, Left Motor Encoder
std_msgs::Int16 encoder_LR_msg;  // Left HBridge, Right Motor Encoder
std_msgs::Int16 encoder_RL_msg;  // Right HBridge, Left Motor Encoder
std_msgs::Int16 encoder_RR_msg;  // Right HBridge, Right Motor Encoder

// Uncomment these if you would like magnetometer, accelerometer, or gyro data.
// geometry_msgs::Vector3 mag_msg;
// geometry_msgs::Vector3 gyro_msg;
// geometry_msgs::Vector3 accel_msg;


// Create Variables for Storing Motion Data
int linear = 0;   // Forward and Backward motion.
int angular = 0;  // Turning (Right and Left) motion.


// Create Publishers
ros::Publisher pub_ir_center("/scan_center", &scan_center_msg);  // Center IR
ros::Publisher pub_ir_back("/scan_back", &scan_back_msg);        // Back IR
ros::Publisher pub_ir_right("/scan_right", &scan_right_msg);     // Right IR
ros::Publisher pub_ir_left("/scan_left", &scan_left_msg);        // Left IR
ros::Publisher pub_ir_cr("/scan_cr", &scan_cr_msg);          // Center Right IR
ros::Publisher pub_ir_cl("/scan_cl", &scan_cl_msg);          // Center Left IR
ros::Publisher pub_encoder_LL("/encoder_LL", &encoder_LL_msg);  // Encoder LL
ros::Publisher pub_encoder_LR("/encoder_LR", &encoder_LR_msg);  // Encoder LR
ros::Publisher pub_encoder_RL("/encoder_RL", &encoder_RL_msg);  // Encoder RL
ros::Publisher pub_encoder_RR("/encoder_RR", &encoder_RR_msg);  // Encoder RR

// Uncomment these if you would like magnetometer, accelerometer, or gyro data.
// ros::Publisher pub_mag("/mag", &mag_msg);        // Magnetometer
// ros::Publisher pub_gyro("/gyro", &gyro_msg);     // Gyroscope
// ros::Publisher pub_accel("/accel", &accel_msg);  // Accelerometer


// Callback for cmd_vel Subscriber.
// The following callback receives a command in terms of m/s. This will have to
// be converted into a rad/s output between -40-40 rad/s.
void callback(const geometry_msgs::Twist &msg) {
  if (msg.linear.x > 0 || msg.linear.x < 0) {
    linear = 40 * msg.linear.x;
    angular = 0;

  } else if (msg.linear.x == 0) {
    linear = 0;

  }

  if (msg.angular.z > 0 || msg.angular.z < 0) {
    linear = 0;
    angular = 40 * msg.angular.z;

  } else if (msg.angular.z == 0) {
    angular = 0;

  }

}


// Create Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", callback);


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

  // Uncomment these if you would like magnetometer, accelerometer, or gyro data.
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
  scan_center_msg.data = pheeno_robot.CDistance;
  scan_back_msg.data = pheeno_robot.BDistance;
  scan_right_msg.data = pheeno_robot.RDistance;
  scan_left_msg.data = pheeno_robot.LDistance;
  scan_cr_msg.data = pheeno_robot.RFDistance;
  scan_cl_msg.data = pheeno_robot.LFDistance;
  encoder_LL_msg.data = pheeno_robot.encoderCountLL;
  encoder_LR_msg.data = pheeno_robot.encoderCountLR;
  encoder_RL_msg.data = pheeno_robot.encoderCountRL;
  encoder_RR_msg.data = pheeno_robot.encoderCountRR;

  // Uncomment these if you would like magnetometer, accelerometer, or gyro data.
  // mag_msg.x = pheeno_robot.IMUMAGX;
  // mag_msg.y = pheeno_robot.IMUMAGY;
  // mag_msg.z = pheeno_robot.IMUMAGZ;
  // accel_msg.x = pheeno_robot.IMUACCX;
  // accel_msg.y = pheeno_robot.IMUACCY;
  // accel_msg.z = pheeno_robot.IMUACCZ;
  // gyro_msg.x = pheeno_robot.IMUGYROX;
  // gyro_msg.y = pheeno_robot.IMUGYROY;
  // gyro_msg.z = pheeno_robot.IMUGYROZ;

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

  // Uncomment these if you would like magnetometer, accelerometer, or gyro data.
  // pub_mag.publish(&mag_msg);
  // pub_accel.publish(&accel_msg);
  // pub_gyro.publish(&gyro_msg);


  if (linear != 0) {
    if (linear > 0) {
      PheenoMoveForward(linear);

    } else {
      PheenoMoveReverse(linear);

    }

  } else if (angular != 0) {
    if (angular > 0) {
      PheenoTurnRight(angular);

    } else {
      PheenoTurnLeft(angular);
    }

  } else {
    pheeno_robot.brakeAll();

  }

  nh.spinOnce();
  delay(100);  // Required because the Teensy sends messages too fast.

}


// Turns the Pheeno left.
// Currently the appropriate use is just by providing a speed in rad/s,
// without any error handling. Be careful! (RANGE: -40 to 40 rad/s)
void PheenoTurnLeft(int speed) {
  pheeno_robot.PIDMotorControlLR(-1 * abs(speed));
  pheeno_robot.PIDMotorControlRL(abs(speed));

}


// Turns the Pheeno Right.
// Currently, the appropriate use is just by providing a speed in rad/s,
// without any error handling. Be careful! (RANGE: -40 to 40 rad/s)
void PheenoTurnRight(int speed) {
  pheeno_robot.PIDMotorControlLR(abs(speed));
  pheeno_robot.PIDMotorControlRL(-1 * abs(speed));

}


// Moves the Pheeno Forward.
// Applying a specific speed value (in rad/s) and both motors will apply the speed
// without error handling. Be careful! (RANGE: -40 to 40 rad/s)
void PheenoMoveForward(int speed) {
  pheeno_robot.PIDMotorControlLR(speed);
  pheeno_robot.PIDMotorControlRL(speed);

}


// Moves the Pheeno Reverse.
// Applying a specific speed value (in rad/s) and both motors will apply the speed
// without error handling. Be careful! (RANGE: -40 to 40 rad/s)
void PheenoMoveReverse(int speed) {
  pheeno_robot.PIDMotorControlLR(-1 * speed);
  pheeno_robot.PIDMotorControlRL(-1 * speed);

}
