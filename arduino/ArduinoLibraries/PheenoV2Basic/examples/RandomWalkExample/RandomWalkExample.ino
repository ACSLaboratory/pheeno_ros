/* This script gives an introduction to making Pheeno drive a random walk. It uses delays and
arduino 8 bit input (int, 0-255) for speed.
*/

#include <PheenoV2Basic.h>

PheenoV2Basic myRobot(1);// The argument is the type of robot: 
                           // 0 = User input parameters (in the c++ library)
                           // 1 = Standard differential drive with 50:1 Gear Motors

float turnDelayTime;//How long the robot will turn.
float runDelayTime;//How long the robot will go straight.
                           
void setup(){
  myRobot.SetupBasic(); //This must be included in every script that uses the Pheeno library! 
                        //It sets up all the pins on the robot and starts relevant timers.
}

void loop(){
  //The user must use the correct commands depending on which connectors the motors are connected to.  
  //the input to all the robot commands must be betwen 0-255.
  
  turnDelayTime = random(0,2000); //randomly generate the ms of delay for the turn.
  if (random(0,2) < 1){ /*The arduino random function is inclusive on the lower bound and exclusive
                        on the upper bound. Thus this randomly generates 0 and 1 and creates a 
                        coin flip whether to move left or right.*/
    myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
    myRobot.reverseLR(70);
    delay(turnDelayTime); //Wait a random amount of time defined above.
  }
  else{
    myRobot.forwardLR(70);//Pheeno turns right about its center at a given speed (range 0 to 255).
    myRobot.reverseRL(70);
    delay(turnDelayTime);//Wait a random amount of time defined above.
  }
  
  runDelayTime = 1000;//ms of delay for the run (you can make this random if you desire!).
  myRobot.forwardLR(150);//Pheeno drives forward at a given speed (range 0 to 255).. 
  myRobot.forwardRL(150);
  delay(runDelayTime);//Wait a random amount of time defined above.
}
