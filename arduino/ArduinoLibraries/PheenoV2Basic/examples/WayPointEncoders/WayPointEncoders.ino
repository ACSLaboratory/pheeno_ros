/* This script makes Pheeno drive to different way points using only
encoder feedback!
*/

#include <PheenoV2Basic.h> // If you want to use Pheeno's premade routines import these libraries!!

PheenoV2Basic myRobot(1); // The argument is the type of robot

float timeStep = 100; //Time step for the system to operate at (10Hz)!
int count = 0;//iterator through the way points.
float desVel = 10;//Desired linear velocity of Pheeno in cm/s.

// Rotational Controller Gains!
float kpAng = 0.932; 
float kiAng = 0.548;
float kdAng = 0;
float rotationalDigitalK = 2.5;

///////////////////////////////////////////////////////////
//PID Way Point Control Constants
///////////////////////////////////////////////////////////

float PIDWayPointTimeStart;

// Error for PIDWayPointControl
float errorAng;
float integralAng;
float diffAng;
float oldErrorAng;

//Control Variable for PIDWayPointControl
float PIDWayPointW;

//WayPoint Path Points.
float botYf[4]={0,0,75.0000,75.00}; //Pheeno's array of waypoint x positions.
float botXf[4]={0,75.0000,75.0000,0}; //Pheeno's array of waypoint y positions.

int numWayPoints = (sizeof(botXf)/sizeof(float));//Number of waypoints entered.

void setup(){
  myRobot.SetupBasic(); //This must be included in every script! It sets up all the pins on the robot!   
  colorWipe(myRobot.LED.Color(255,0,0),100);//Red Wipe
  colorWipe(myRobot.LED.Color(0,0,255),100);//Blue Wipe
  PIDWayPointTimeStart=millis();//Set the PID controller timer initially.
}

void loop() {
  while (calculateDistance(myRobot.botXPos,myRobot.botYPos,botXf[count%numWayPoints],botYf[count%numWayPoints]) > 3){
    PIDWayPointControl(botXf[count%numWayPoints],botYf[count%numWayPoints],desVel,timeStep);
    myRobot.encoderPositionUpdate(10); //Encoders used for state estimates.
  }
  count ++;
}

float calculateDistance(float x1, float y1, float x2, float y2){
  //Calculates the distance the robot is from the desired way point
  float space = sqrt(sq((x1 - x2)) + sq((y1 - y2)));
  return space;
}

void PIDWayPointControl(float u1, float u2, float desVelocity, float timeStep){
  /*Causes Pheeno to move to waypoint located at (u1,u2) assuming inial
  condition X0,Y0,A0 defined initially and updated by the robot's
  odometry.*/

  if (millis() - PIDWayPointTimeStart >= timeStep){
    float PIDTimeStep = (millis()-PIDWayPointTimeStart)/1000;
    
    float desHeading = atan2((u2-myRobot.botYPos), (u1-myRobot.botXPos));// rad
    
    //Calculate Errors for PID control
    errorAng = wrapToPi(desHeading - myRobot.botA); 
    integralAng = wrapToPi(integralAng + errorAng * PIDTimeStep);
    diffAng = wrapToPi((oldErrorAng - errorAng) / PIDTimeStep);
    oldErrorAng = errorAng;  
     
    PIDWayPointW = rotationalDigitalK*(kpAng*errorAng + kiAng*integralAng + kdAng*diffAng);
      
    PIDWayPointTimeStart = millis(); 
  }
  // motorLVel and motorRVel are in rad/s
  float motorLVel = convertUnicycleToLeftMotor(desVelocity, PIDWayPointW);
  float motorRVel = convertUnicycleToRightMotor(desVelocity, PIDWayPointW); 

  myRobot.PIDMotorControlLR(motorLVel); //Controlled Rotational Shaft Velocity of Right Motor on Left Hbridge (rad/s)
  myRobot.PIDMotorControlRL(motorRVel); //Controlled Rotational Shaft Velocity of Left Motor on Right Hbridge (rad/s)
}

///////////////////////////////////////////////////////////
// Unicycle Model Conversions
///////////////////////////////////////////////////////////

float convertUnicycleToRightMotor(float vel, float w){
  float output = (2.0*vel+w*myRobot.axelLength)/(myRobot.wheelDiameter);
  return output;
}

float convertUnicycleToLeftMotor(float vel, float w){
  float output = (2.0*vel-w*myRobot.axelLength)/(myRobot.wheelDiameter);
  return output;
}

///////////////////////////////////////////////////////////
//Conversion Functions
///////////////////////////////////////////////////////////

float wrapToPi(float rad){
  //Converts radian angle (-pi,pi]
  float output = atan2(sin(rad),cos(rad));
  return output;
}

///////////////////////////////////////////////////////////
//Fun Light Functions
///////////////////////////////////////////////////////////
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<myRobot.LED.numPixels(); i++) {
    myRobot.LED.setPixelColor(i, c);
    myRobot.LED.show();
    delay(wait);
  } 
}
