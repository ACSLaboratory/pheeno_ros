/* This script gives an introduction to making Pheeno drive open loop.
There is no feedback, motors move at the given input but can be prone
to errors in manufacturing and environment. Everything will be done with
delays, however, it should be noted you cannot use delays 
if you require sensor feedback.
*/

#include <PheenoV2Basic.h>

PheenoV2Basic myRobot(1);// The argument is the type of robot: 
                           // 0 = User input parameters (in the c++ library)
                           // 1 = Standard differential drive with 50:1 Gear Motors

void setup(){
  myRobot.SetupBasic(); //This must be included in every script that uses the Pheeno library! 
                        //It sets up all the pins on the robot and starts relevant timers.
}

void loop(){
  //The user must use the correct commands depending on which connectors the motors are connected to.  
  //the input to all the robot commands must be betwen 0-255.
  
  myRobot.forwardLR(150);//Pheeno drives forward at a given speed (range 0 to 255).. 
  myRobot.forwardRL(150);
  delay(5000);//Wait for 5 seconds
  myRobot.forwardLR(70);//Pheeno turns right about its center at a given speed (range 0 to 255).
  myRobot.reverseRL(70);
  delay(2000);//Wait for 2 seconds.
  myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
  myRobot.reverseLR(70);
  delay(2000);//Wait for 2 seconds.
  myRobot.reverseLR(150);//Pheeno drives backwards at a given speed (range 0 to 255). 
  myRobot.reverseRL(150);
  delay(5000);//Wait for 5 seconds
  myRobot.brakeAll();//Brake the motors.
  delay(3000);//Wait for 3 seconds.
}
