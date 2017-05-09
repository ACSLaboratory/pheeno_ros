#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "PheenoV2Basic.h"

// Create ROS Node instance
ros::NodeHandle nh;

// Create Pheeno Instance
PheenoV2Basic pheeno_robot = PheenoV2Basic(2);

// Create Publishers
ros::Publisher pub_ir_center();  // Center IR Sensor
ros::Publisher pub_ir_back();    // Back IR Sensor
ros::Publisher pub_ir_right();   // Right IR Sensor
ros::Publisher pub_ir_left();    // Left IR Sensor
ros::Publisher pub_ir_cr();      // Center Right IR Sensor
ros::Publisher pub_ir_cl();      // Center Left IR Sensor
ros::Publisher pub_mag();        // Magnetometer
ros::Publisher pub_gyro();       // Gyroscope
ros::Publisher pub_accel();      // Accelerometer

// Callback for Subscribers
void callback(const geometry_msgs::Twist &msg) {
  if (msg.linear.x > 0) {
    PheenoMoveForward();
  } else if (msg.linear.x < 0) {
    PheenoMoveReverse();
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
  nh.advertise(pub_mag);
  nh.advertise(pub_gyro);
  nh.advertise(pub_accel);

  // Start Subscribing
  nh.subscribe(sub_cmd_vel);

}


void loop() {
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
