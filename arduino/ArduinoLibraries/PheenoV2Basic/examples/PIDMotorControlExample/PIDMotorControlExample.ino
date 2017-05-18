/* This script gives an introduction to using encoder feedback on Pheeno's motors.
In this code we will make Pheeno drive straight at a given velocity.
A premade PID controller has been made to control the individual motor speeds
and can be accessed using the PIDMotorControl() function. The PID feedback
relies solely on the encoders so wheel slip can cause to robot to diverge 
from a straight line path! The gains of the PID can be adjusted in the Pheeno.cpp file.
*/

#include <PheenoV2Basic.h>

PheenoV2Basic myRobot(1);// The argument is the type of robot: 
                           // 0 = User input parameters (in the c++ library)
                           // 1 = Standard differential drive with 50:1 Gear Motors
                           
float desVel = -15; //The linear velocity we want the robot to go in cm/s

void setup(){
  myRobot.SetupBasic(); //This must be included in every script that uses the Pheeno library! 
                        //It sets up all the pins on the robot and starts relevant timers.
  desVel = desVel * 2/myRobot.wheelDiameter;//Converts the linear velocity to rotational velocity for the controller.
}

void loop() {
  //The user must use the correct commands depending on which connectors the motors are connected to.  
  
  myRobot.PIDMotorControlLR(desVel);//Pheeno drives forward. 
  myRobot.PIDMotorControlRR(desVel);
}
