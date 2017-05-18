/* This script gives an introduction to making Pheeno drive a random walk. 
While doing the random walk, it will avoid obstacles.
*/

#include <PheenoV2Basic.h> // If you want to use Pheeno's premade routines import these libraries!!

PheenoV2Basic myRobot(1); // The argument is the type of robot

float pi = 3.1416;
float avoidDist = 25; //Distance in CM at which Pheeno will avoid obstacles.
float randomWalkTimeStep = 5000; //Time to walk forward.
float timeStartRandomWalk;
float desHeading; //Desired heading for the random walk.
float desHeadingAvoid; //Desired heading for the random walk.
float const desVel = 10; //cm/s
float northOffset = 0;//rad
bool avoid = false;//Whether to follow avoidance heading.

// Rotational Controller Gains!
float const kpAng = 0.632; 
float const kiAng = 1.248;
float const kdAng = 0;
float const rotationalDigitalK = 2.5;

// Error for PIDHeadingControl
float errorAng = 0;
float integralAng = 0;
float diffAng = 0;
float oldErrorAng = 0;
float PIDHeadingControlTimeStart;
float PIDWayPointW;

//Distance sensor variables for this entire script; L=Left, LF=Left forward, C=Center, RF=Right forward, R=Right, B=Back.
float dL;
float dLF;
float dC;
float dRF;
float dR;
float dB;
float dSumL;//The sum of all the left side distances.
float dSumR;//The sum of all the right side distances.
float dSumC;//The sum of all the center distances.

float LSD;//average distance for sides
float RSD;
float CD;

//Angles the sensors are placed on the robot.
float aL = pi/2;
float aLF = pi/4;
float aC = 0;
float aRF = -pi/4;
float aR = -pi/2;
float aB = pi;

//Unit Vectors in the direction of the sensors on the robot.
float vLX = cos(aL);
float vLY = sin(aL);

float vLFX = cos(aLF);
float vLFY = sin(aLF);

float vCX = cos(aC);
float vCY = sin(aC);

float vRFX = cos(aRF);
float vRFY = sin(aRF);

float vRX = cos(aR);
float vRY = sin(aR);

float vBX = cos(aB);
float vBY = sin(aB);

float wVXL;//weighted sum vector in the x direction.
float wVYL;//weighted sum vector in the y direction.

float wVXR;//weighted sum vector in the x direction.
float wVYR;//weighted sum vector in the y direction.

float wVXC;//weighted sum vector in the x direction.
float wVYC;//weighted sum vector in the y direction.

void setup(){  
  myRobot.SetupBasic(); //This must be included in every script! It sets up all the pins on the robot!
  for(int i=1;i<1000;i++){
    myRobot.readMag(0);
    northOffset = (i-1)/i * northOffset + 1/i * myRobot.IMUHeading;
  }
  colorWipe(myRobot.LED.Color(255,0,0),100);
  colorWipe(myRobot.LED.Color(255,255,0),100);
  colorWipe(myRobot.LED.Color(0,255,0),100);
  colorWipe(myRobot.LED.Color(0,255,255),100);
  colorWipe(myRobot.LED.Color(255,0,255),100);

  timeStartRandomWalk = millis();
  PIDHeadingControlTimeStart = millis();
}

void loop(){  
  avoidObstacles();
  myRobot.sensorFusionPositionUpdate(northOffset);
  if (millis() - timeStartRandomWalk >= randomWalkTimeStep){
    chooseRandomHeading();
  }

  if (avoid){
    PIDHeadingControl(desHeadingAvoid, desVel, 100); // Forward and maintain heading
  }
  else{
    PIDHeadingControl(desHeading, desVel, 100);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//MOTION SUB ROUTINES
///////////////////////////////////////////////////////////////////////////////////////////////////

void chooseRandomHeading(){
  desHeading = wrapToPi(random(-180,180) * pi/180);
  timeStartRandomWalk = millis();
}

void avoidObstacles(){
  myRobot.readIR();//Read all the IR range sensors on the robot.
  
  if (myRobot.LDistance > avoidDist && myRobot.LFDistance > avoidDist && myRobot.CDistance > avoidDist && myRobot.RFDistance > avoidDist && myRobot.RDistance > avoidDist){
    avoid = false;
    return;//Nothing to avoid so leave the loop.  
  } 
  
  /*Segment the IR sensors into 3 groups Left Side, Center, Right Side*/
  dL = 1/myRobot.LDistance;
  dLF = 1/myRobot.LFDistance;
  dC = 1/myRobot.CDistance;
  dRF = 1/myRobot.RFDistance;
  dR = 1/myRobot.RDistance;
  dB = 1/myRobot.BDistance;

  if (myRobot.LDistance > 40){
    dL = 0;
  }
  if (myRobot.LFDistance > 40){
    dLF = 0;
  }
  if (myRobot.CDistance > 40){
    dC = 0;
  }
  if (myRobot.RFDistance > 40){
    dRF = 0;
  }
  if (myRobot.RDistance > 40){
    dR = 0;
  }
  if (myRobot.BDistance > 40){
    dB = 0;
  }

  dSumL = dL + dLF;
  dSumC = dLF + dC + dRF;
  dSumR = dR + dRF;

  wVXL=40;
  wVYL=40;

  wVXR=40;
  wVYR=40;

  wVXC=40;
  wVYC=40;
  
  if (dSumL > 0){
    wVXL = dL/dSumL * myRobot.LDistance * vLX + dLF/dSumL * myRobot.LFDistance * vLFX;
    wVYL = dL/dSumL * myRobot.LDistance * vLY + dLF/dSumL * myRobot.LFDistance * vLFY;
  }

  if (dSumC > 0){
    wVXC = dC/dSumC * myRobot.CDistance * vCX + dRF/dSumC * myRobot.RFDistance * vRFX + dLF/dSumC * myRobot.LFDistance * vLFX;  
    wVYC = dC/dSumC * myRobot.CDistance * vCY + dRF/dSumC * myRobot.RFDistance * vRFY + dLF/dSumC * myRobot.LFDistance * vLFY;
  }
  
  if (dSumR > 0){
    wVXR = dR/dSumR * myRobot.RDistance * vRX + dRF/dSumR * myRobot.RFDistance * vRFX;
    wVYR = dR/dSumR * myRobot.RDistance * vRY + dRF/dSumR * myRobot.RFDistance * vRFY;
  }

  LSD = sqrt(pow(wVYL,2) + pow(wVXL,2));
  RSD = sqrt(pow(wVYR,2) + pow(wVXR,2));
  CD = sqrt(pow(wVYC,2) + pow(wVXC,2));

  if (CD <= LSD && CD <= RSD){
    wVXC = dC/dSumC * vCX + dRF/dSumC * vRFX + dLF/dSumC * vLFX;
    wVYC = dC/dSumC * vCY + dRF/dSumC * vRFY + dLF/dSumC * vLFY;

    desHeadingAvoid = myRobot.botA + wrapToPi(atan2(wVYC,wVXC) + pi/2);//In radians
    desHeading = desHeadingAvoid;
    timeStartRandomWalk = millis();
    avoid = true;
    return;
  }
  else if (LSD < CD && LSD <= RSD){
    wVXL = dL/dSumL * vLX + dLF/dSumL * vLFX;
    wVYL = dL/dSumL * vLY + dLF/dSumL * vLFY;

    desHeadingAvoid = myRobot.botA + wrapToPi(atan2(wVYL,wVXL) + pi/2);//In radians
    avoid = true;
    desHeading = desHeadingAvoid;
    timeStartRandomWalk = millis();
    return;
  }
  else if (RSD < CD && RSD < LSD){
    wVXR = dR/dSumR * vRX + dRF/dSumR * vRFX;
    wVYR = dR/dSumR * vRY + dRF/dSumR * vRFY;

    desHeadingAvoid = myRobot.botA + wrapToPi(atan2(wVYR,wVXR) + pi/2);//In radians
    desHeading = desHeadingAvoid;
    timeStartRandomWalk = millis();
    avoid = true;
    return;
  }
}

void PIDHeadingControl(float desH, float desV, float timeStep){  
  if (millis() - PIDHeadingControlTimeStart >= timeStep){
    float PIDTimeStep = (millis()-PIDHeadingControlTimeStart)/1000;    
    
    //Calculate Errors for PID control
    errorAng = wrapToPi(desH - myRobot.botA); 
    integralAng = wrapToPi(errorAng * PIDTimeStep);
    diffAng = wrapToPi((oldErrorAng - errorAng) / PIDTimeStep);
    oldErrorAng = errorAng;  
     
    PIDWayPointW = rotationalDigitalK*(kpAng*errorAng + kiAng*integralAng + kdAng*diffAng);
      
    PIDHeadingControlTimeStart = millis(); 
  }
  // motorLVel and motorRVel are in rad/s
  float motorLVel = convertUnicycleToLeftMotor(desV, PIDWayPointW);
  float motorRVel = convertUnicycleToRightMotor(desV, PIDWayPointW);

  myRobot.PIDMotorControlRL(motorRVel);
  myRobot.PIDMotorControlLR(motorLVel);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<myRobot.LED.numPixels(); i++) {
    myRobot.LED.setPixelColor(i, c);
    myRobot.LED.show();
    delay(wait);
  } 
}

float deg2rad(float deg){
  //Converts degree to radians
  float output = deg * pi / 180;
  return output;
}

float rad2deg(float rad){
  //Converts radians to degree
  float output = rad * 180 / pi;
  return output;
}

float wrapToPi(float rad){
  //Converts radian angle (-pi,pi]
  float output = atan2(sin(rad),cos(rad));
  return output;
}

float wrapTo2Pi(float rad){
  //Converts radian angle [0,2*pi)
  float output = wrapToPi(rad);
  if (output < 0){
    output = output + 2*pi;
  }
  return output;
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
