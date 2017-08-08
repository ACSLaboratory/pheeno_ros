/* This script gives an introduction to making Pheeno drive a random walk.
While doing the random walk, it will avoid obstacles.
*/

#include <PheenoV2Basic.h>

PheenoV2Basic myRobot(1);// The argument is the type of robot:
                           // 0 = User input parameters (in the c++ library)
                           // 1 = Standard differential drive with 50:1 Gear Motors

float rangeToAvoid = 10; //Distance in CM at which Pheeno will avoid obstacles (must be >4 and <25).
float compareTime; //Compare time in ms we will base our while loops off of. Initialized in the set up!

float turnDelayTime; //How long the robot will turn for its random walk.
float runDelayTime; //How long the robot will drive straight for its random walk.

void setup() {
  myRobot.SetupBasic(); //This must be included in every script that uses the Pheeno library!
                        //It sets up all the pins on the robot and starts relevant timers.
  compareTime = millis(); //Compare time in ms we will base our while loops off of.
}

void loop() {
  turnDelayTime = random(0, 2000); //randomly generate the ms of delay for the turn.
  if (random(0, 2) < 1) { /*The arduino random function is inclusive on the lower bound and exclusive
                        on the upper bound. Thus this randomly generates 0 and 1 and creates a
                        coin flip whether to move left or right.*/

    myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
    myRobot.reverseLR(70);

    while(millis() - compareTime < turnDelayTime) { //Wait a random amount of time defined above.
      avoidObstacles();//While turning still looking for obstacles!
    }
    compareTime = millis();

  } else {
    myRobot.forwardLR(70);//Pheeno turns right about its center at a given speed (range 0 to 255).
    myRobot.reverseRL(70);

    while(millis() - compareTime < turnDelayTime) { //Wait a random amount of time defined above.
      avoidObstacles();//While turning still looking for obstacles!
    }
    compareTime=millis();
  }

  runDelayTime = 1000;//ms of delay for the run (you can make this random if you desire!).

  myRobot.forwardLR(150);//Pheeno drives forward at a given speed (range 0 to 255)..
  myRobot.forwardRL(150);

  while(millis() - compareTime < runDelayTime) { //Wait a random amount of time defined above.
      avoidObstacles();//While driving forward, still looking for obstacles to not run into.
  }
  compareTime = millis();
}

void avoidObstacles() {
  myRobot.readIR();
  int collisionRotateSpeed = 150; //Speed at which to rotate away from collisions (In arduino PWM units (int, 0 to 255)).
  if (myRobot.CDistance < rangeToAvoid) {
    if(abs((myRobot.RDistance - myRobot.LDistance) < 5 ||
        (myRobot.RDistance > rangeToAvoid && myRobot.LDistance > rangeToAvoid)) ) {
      if(random(0,2) < 1) {
        myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
        myRobot.reverseLR(70);
      } else {
        myRobot.forwardLR(collisionRotateSpeed);//Pheeno turns right about its center at a given speed (range 0 to 255).
        myRobot.reverseRL(collisionRotateSpeed);
      }
    }

    if (myRobot.RDistance < myRobot.LDistance) {
      myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
      myRobot.reverseLR(70);
    } else {
      myRobot.forwardLR(collisionRotateSpeed);//Pheeno turns right about its center at a given speed (range 0 to 255).
      myRobot.reverseRL(collisionRotateSpeed);
    }

  } else if (myRobot.RFDistance < rangeToAvoid && myRobot.LFDistance < rangeToAvoid) {
    if(random(0,2) < 1){
      myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
      myRobot.reverseLR(70);
    } else {
      myRobot.forwardLR(collisionRotateSpeed);//Pheeno turns right about its center at a given speed (range 0 to 255).
      myRobot.reverseRL(collisionRotateSpeed);
    }

  } else if (myRobot.RFDistance < rangeToAvoid) {
    myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
    myRobot.reverseLR(70);

  } else if (myRobot.LFDistance < rangeToAvoid) {
    myRobot.forwardLR(collisionRotateSpeed);//Pheeno turns right about its center at a given speed (range 0 to 255).
    myRobot.reverseRL(collisionRotateSpeed);

  } else if (myRobot.LDistance < rangeToAvoid) {
    myRobot.forwardLR(collisionRotateSpeed);//Pheeno turns right about its center at a given speed (range 0 to 255).
    myRobot.reverseRL(collisionRotateSpeed);
  } else if (myRobot.RDistance < rangeToAvoid) {
    myRobot.forwardRL(70);//Pheeno turns left about its center at a given speed (range 0 to 255).
    myRobot.reverseLR(70);
  }

}
