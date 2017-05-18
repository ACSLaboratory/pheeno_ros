#include "PheenoV2Basic.h"

PheenoV2Basic::PheenoV2Basic(int type):_motorLL(_encoder1LL, _encoder2LL),\
_motorLR(_encoder1LR, _encoder2LR),_motorRL(_encoder1RL, _encoder2RL),\
_motorRR(_encoder1RR, _encoder2RR), LED(8, _LED, NEO_GRB + NEO_KHZ800){
  ///////////////////////////////////////////////////////////
  //Set kinematic measurements and PID Gains based on base type.
  ///////////////////////////////////////////////////////////

  // The argument is the type of robot 
  if (type == 0){ 
    //User has chosen to set the kinematic constants, encoder count constant, 
    //and motor control gains themselves.
  }
  else if (type == 1){
    //User has chosen the constants associated with the standard rear differential
    //drive version of the robot with the Pololu 50:1 micro metal geared motors.

    encoderCountsPerRotation = 12;
    motorGearRatio = 51.45; // The gearing ratio of the drive motor being used.
    wheelDiameter = 3.2; // Wheel Diameter in cm.
    axelLength = 12.9; // Axel length in cm.

    //Motor PID Control Constants
    _kpMotor = 5.16;
    _kiMotor = 75;//20*_kpMotor;//123.9;
    _kdMotor = 0;  
  }
}

///////////////////////////////////////////////////////////
//Setup
///////////////////////////////////////////////////////////

void PheenoV2Basic::SetupBasic(){
  Wire.begin();//Start Communication to IMU
  LED.begin();//Initialize NeoPixel Strip
  for(uint16_t i=0; i<LED.numPixels(); i++) {
    LED.setPixelColor(i, (0,0,0));//Turn off NeoPixels
  }

  //Initialize the Magnetometer, Accelerometer, and Gyro. If something
  //Goes wrong it sets the first NeoPixel red to alert the user.
  if (!mag.init())
  {
    LED.setPixelColor(0,255,0,0);
  }
  else
  {
    mag.enableDefault();    
  }
  
  if (!imu.init())
  {
    LED.setPixelColor(0,255,0,0);
  }
  else
  {
    imu.enableDefault();
  }

  //Get the calibration data from the EEPROM at position 'eeAddress'
  EEPROM.get(_eeAddress, _accPitchCalibration);
  _eeAddress += sizeof(float);
  EEPROM.get(_eeAddress, _accRollCalibration);
  _eeAddress += sizeof(float);
  EEPROM.get(_eeAddress, _accYawCalibration);
  _eeAddress += sizeof(float);
  EEPROM.get(_eeAddress, _gyroOffX);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _gyroOffY);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _gyroOffZ);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _magOffX);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _magOffY);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _magOffZ);
  _eeAddress += sizeof(int);
  EEPROM.get(_eeAddress, _magScaleX);
  _eeAddress += sizeof(float);
  EEPROM.get(_eeAddress, _magScaleY);
  _eeAddress += sizeof(float);
  EEPROM.get(_eeAddress, _magScaleZ);

  LED.show();//Initialize all NeoPixels to OFF or First to RED if
             //there is an issue initializing the IMU.

  ///////////////////////////////////////////////////////////
  //Set Pin Modes!
  ///////////////////////////////////////////////////////////
  
  //IR Sensor Input Pins (Analog)
  pinMode(_IRC, INPUT);//Set the Center IR Pin to Input
  pinMode(_IRLF, INPUT);//Set the Left Forward IR Pin to Input
  pinMode(_IRL, INPUT);//Set the Left IR Pin to Input
  pinMode(_IRB, INPUT);//Set the Back IR Pin to Input
  pinMode(_IRR, INPUT);//Set the Right IR Pin to Input
  pinMode(_IRRF, INPUT);//Set the Right Forward IR Pin to Input

  //Batt Voltage Input
  pinMode(_BattVolt, INPUT);//Set the Battery Voltage Sensing Pin to Input

  //Camera Servo Control
  pinMode(_CameraServo, OUTPUT);//Set the Camera Servo Control Pin to Output

  //NeoPixels
  pinMode(_LED, OUTPUT);//Set the NeoPixel Control Pin to Output

  //Motor Control
  pinMode(_PWMLL, OUTPUT);//Set the Left HBridge, Left Motor PWM Control Pin to Output
  pinMode(_PWMLR, OUTPUT);//Set the Left HBridge, Right Motor PWM Control Pin to Output
  pinMode(_PWMRL, OUTPUT);//Set the Right HBridge, Left Motor PWM Control Pin to Output
  pinMode(_PWMRR, OUTPUT);//Set the Right HBridge, Right Motor PWM Control Pin to Output

  pinMode(_LLMotorDirection1, OUTPUT);// Set the Left HBridge, Left Motor Direction 1 Pin to Output
  pinMode(_LLMotorDirection2, OUTPUT);// Set the Left HBridge, Left Motor Direction 2 Pin to Output
  pinMode(_LRMotorDirection1, OUTPUT);// Set the Left HBridge, Right Motor Direction 1 Pin to Output
  pinMode(_LRMotorDirection2, OUTPUT);// Set the Left HBridge, Right Motor Direction 2 Pin to Output
  pinMode(_RLMotorDirection1, OUTPUT);// Set the Right HBridge, Left Motor Direction 1 Pin to Output
  pinMode(_RLMotorDirection2, OUTPUT);// Set the Right HBridge, Left Motor Direction 2 Pin to Output
  pinMode(_RRMotorDirection1, OUTPUT);// Set the Right HBridge, Right Motor Direction 1 Pin to Output
  pinMode(_RRMotorDirection2, OUTPUT);// Set the Right HBridge, Right Motor Direction 2 Pin to Output

  pinMode(_LSTBY, OUTPUT);//Set the Standby pin to turn off motors attached to Left HBridge to output.
  pinMode(_RSTBY, OUTPUT);//Set the Standby pin to turn off motors attached to Right HBridge to output.

  _PIDMotorsTimeStartLL = millis();//Start time for motor controller to operate on correct delay (Left Hbridge, Left Motor).
  _PIDMotorsTimeStartLR = millis();//Start time for motor controller to operate on correct delay (Left Hbridge, Right Motor).
  _PIDMotorsTimeStartRL = millis();//Start time for motor controller to operate on correct delay (Right Hbridge, Left Motor).
  _PIDMotorsTimeStartRR = millis();//Start time for motor controller to operate on correct delay (Right Hbridge, Right Motor).

  _positionUpdateTimeStart = millis();
}

///////////////////////////////////////////////////////////
//Private Conversion Functions
///////////////////////////////////////////////////////////

float PheenoV2Basic::_deg2rad(float deg){
  //Converts degree to radians
  float output = deg * _pi / 180;
  return output;
}

float PheenoV2Basic::_rad2deg(float rad){
  //Converts radians to degree
  float output = rad * 180 / _pi;
  return output;
}

float PheenoV2Basic::_wrapToPi(float rad){
  //Converts radian angle (-pi,pi]
  float output = atan2(sin(rad),cos(rad));
  return output;
}

float PheenoV2Basic::_wrapTo2Pi(float rad){
  //Converts radian angle [0,2*pi)
  float output = _wrapToPi(rad);
  if (output < 0){
    output = output + 2*_pi;
  }
  return output;
}

void PheenoV2Basic::_calibrationAccRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz){
  //Rotates measurement about calibration angles found in accelerometer calibration. 
  //Rotates accelerometer reference frame to Pheenos.

  /* Ry(roll)Rx(pitch)Rz(yaw) = [cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw), sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), -sin(roll)*cos(pitch);
  *                              -cos(pitch)*sin(yaw), cos(pitch)*cos(yaw), sin(pitch);
  *                              cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw), -cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll), cos(roll)*cos(pitch)]
  */

  IMUACCX = (cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw))*Mx + (sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw))*My + (-sin(roll)*cos(pitch))*Mz;
  IMUACCY = (-cos(pitch)*sin(yaw))*Mx + (cos(pitch)*cos(yaw))*My + (sin(pitch))*Mz;
  IMUACCZ = (cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw))*Mx + (-cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll))*My + (cos(roll)*cos(pitch))*Mz;
}

void PheenoV2Basic::_calibrationGyroRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz){
  //Rotates measurement about calibration angles found in accelerometer calibration. 
  //Rotates accelerometer reference frame to Pheenos.

  /* Ry(roll)Rx(pitch)Rz(yaw) = [cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw), sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), -sin(roll)*cos(pitch);
  *                              -cos(pitch)*sin(yaw), cos(pitch)*cos(yaw), sin(pitch);
  *                              cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw), -cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll), cos(roll)*cos(pitch)]
  */

  IMUGYROX = (cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw))*Mx + (sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw))*My + (-sin(roll)*cos(pitch))*Mz;
  IMUGYROY = (-cos(pitch)*sin(yaw))*Mx + (cos(pitch)*cos(yaw))*My + (sin(pitch))*Mz;
  IMUGYROZ = (cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw))*Mx + (-cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll))*My + (cos(roll)*cos(pitch))*Mz;
}

void PheenoV2Basic::_calibrationMagRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz){
  //Rotates measurement about calibration angles found in accelerometer calibration. 
  //Rotates accelerometer reference frame to Pheenos.

  /* Ry(roll)Rx(pitch)Rz(yaw) = [cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw), sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), -sin(roll)*cos(pitch);
  *                              -cos(pitch)*sin(yaw), cos(pitch)*cos(yaw), sin(pitch);
  *                              cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw), -cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll), cos(roll)*cos(pitch)]
  */

  IMUMAGX = (cos(yaw)*cos(roll)-sin(roll)*sin(pitch)*sin(yaw))*Mx + (sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw))*My + (-sin(roll)*cos(pitch))*Mz;
  IMUMAGY = (-cos(pitch)*sin(yaw))*Mx + (cos(pitch)*cos(yaw))*My + (sin(pitch))*Mz;
  IMUMAGZ = (cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw))*Mx + (-cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll))*My + (cos(roll)*cos(pitch))*Mz;
}

///////////////////////////////////////////////////////////
//Read Sensors
///////////////////////////////////////////////////////////

void PheenoV2Basic::readIR(){ // Gets a distance from IR sensors in cm.
  // Takes the value from the IR and converts it to a voltage.
  
  float voltsR = analogRead(_IRR)* 0.00322265625;   // value from right sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsC = analogRead(_IRC)* 0.00322265625;   // value from center sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsL = analogRead(_IRL)* 0.00322265625;   // value from left sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsLF = analogRead(_IRLF)* 0.00322265625;   // value from left forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsRF = analogRead(_IRRF)* 0.00322265625;   // value from right forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsB = analogRead(_IRB)* 0.00322265625;   // value from right forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  
  
  /* This calibration is for the Sharp GP2Y0A41SK0F IR distance sensor.(http://www.pololu.com/product/2464)
  
  This measures to the back of the sensor! If you want the distance to the lense subtract off the width of the sensor.
    The units are in CM.
  
  This sensor has good accuracy from 4~30 cm then starts to get fuzzy.
  
  D=a/(V-b);
  
  a=15.68;
  b=-0.03907;
 
  Width of the Sensor=1.3;
  */
     RDistance = _a/(voltsR - _b);
     CDistance = _a/(voltsC - _b);
     LDistance = _a/(voltsL - _b);
     LFDistance = _a/(voltsLF - _b);
     RFDistance = _a/(voltsRF - _b);
     BDistance = _a/(voltsB - _b);
}

void PheenoV2Basic::readEncoders(){
  encoderCountLL = -_motorLL.read();
  encoderCountLR = -_motorLR.read();
  encoderCountRL = _motorRL.read();
  encoderCountRR = _motorRR.read();
}

void PheenoV2Basic::readMag(float magNorthOffset){
  //Reads the raw magnetometer values in each direction
  //and converts the field to gauss. Using default conversion 6842 LSB/Gauss.

  mag.read();

  float tempX  = (_magScaleX*(mag.m.x-_magOffX)) / 6842.0;
  float tempY  = (_magScaleY*(mag.m.y-_magOffY)) / 6842.0;
  float tempZ  = (_magScaleZ*(mag.m.z-_magOffZ)) / 6842.0;

  _calibrationMagRotation(_accPitchCalibration, _accRollCalibration, _accYawCalibration, tempX, tempY, tempZ);

  IMUHeading = atan2(IMUMAGY,IMUMAGX) - magNorthOffset;
}

void PheenoV2Basic::readGyro(){
  //Reads the raw gyroscope values about each direction
  //and converts the rates to rad/s. Using default conversion 8.75mdps/LSB = 0.00015272rad/s/LSB.
  imu.readGyro();

  float tempX = -(imu.g.x - _gyroOffX)*0.00015272;//Need negative for RH reference frame.
  float tempY = (imu.g.y - _gyroOffY)*0.00015272;
  float tempZ = -(imu.g.z - _gyroOffZ)*0.00015272;

  _calibrationGyroRotation(_accPitchCalibration, _accRollCalibration, _accYawCalibration, tempX, tempY, tempZ);

}

void PheenoV2Basic::readAccel(){
  //Reads the raw accelerometer values in each direction, corrects the rotational alignment
  //and converts the data to cm/s^2. Using default conversion 0.061mg/LSB = 0.0059841cm/s^2/LSB.
  imu.readAcc();

  float tempX = (imu.a.x) * 0.0059841;
  float tempY = (-imu.a.y) * 0.0059841;//Need a negative here for RH reference frame.
  float tempZ = (imu.a.z) * 0.0059841;

  _calibrationAccRotation(_accPitchCalibration, _accRollCalibration, _accYawCalibration, tempX, tempY, tempZ);
}

///////////////////////////////////////////////////////////
//Motor Functions
///////////////////////////////////////////////////////////

void PheenoV2Basic::noMotionAll(){
  // Turns all the motors off. (They can still rotate passively) 
  digitalWrite(_LSTBY, LOW); //Left HBridge Motors OFF
  digitalWrite(_RSTBY, LOW); //Right HBridge Motors OFF
}

void PheenoV2Basic::brakeAll(){
  //Brakes all the motors. (Locked to not rotate passively, this can be overcome by enough torque)
  digitalWrite(_LSTBY, HIGH);// Left HBridge ON
  digitalWrite(_RSTBY, HIGH);// Right HBridge ON
  
  analogWrite(_PWMLL, LOW);//Left HBridge, Left Motor PWM signal set to zero.
  digitalWrite(_LLMotorDirection1, HIGH);//Left HBridge, Left Motor Direction Pins Shorted.
  digitalWrite(_LLMotorDirection1, HIGH);

  analogWrite(_PWMLR, LOW);//Left HBridge, Right Motor PWM signal set to zero.
  digitalWrite(_LRMotorDirection1, HIGH);//Left HBridge, Right Motor Direction Pins Shorted.
  digitalWrite(_LRMotorDirection1, HIGH);

  analogWrite(_PWMRL, LOW);//Right HBridge, Left Motor PWM signal set to zero.
  digitalWrite(_RLMotorDirection1, HIGH);//Right HBridge, Left Motor Direction Pins Shorted.
  digitalWrite(_RLMotorDirection1, HIGH);

  analogWrite(_PWMRR, LOW);//Right HBridge, Right Motor PWM signal set to zero.
  digitalWrite(_RRMotorDirection1, HIGH);//Right HBridge, Right Motor Direction Pins Shorted.
  digitalWrite(_RRMotorDirection1, HIGH);
}

void PheenoV2Basic::forwardLL(int motorSpeed){
  // Left HBridge, Left motor drive forward.  
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(_LSTBY, HIGH); // Left HBridge Motors ON 
  analogWrite(_PWMLL, motorSpeed); // Provide PWM Created Analog Voltage to Motor
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation. 
  digitalWrite(_LLMotorDirection1, LOW);     
  digitalWrite(_LLMotorDirection2, HIGH);
}

void PheenoV2Basic::forwardLR(int motorSpeed){
  // Left HBridge, Right motor drive forward.  
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(_LSTBY, HIGH); // Left HBridge Motors ON 
  analogWrite(_PWMLR, motorSpeed); // Provide PWM Created Analog Voltage to Motor
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation.
  digitalWrite(_LRMotorDirection1, HIGH);     
  digitalWrite(_LRMotorDirection2, LOW);
}

void PheenoV2Basic::forwardRL(int motorSpeed){
  // Right HBridge, Left motor drive forward.  
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(_RSTBY, HIGH); // Right HBridge Motors ON 
  analogWrite(_PWMRL, motorSpeed); // Provide PWM Created Analog Voltage to Motor
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation. 
  digitalWrite(_RLMotorDirection1, HIGH);     
  digitalWrite(_RLMotorDirection2, LOW);
}

void PheenoV2Basic::forwardRR(int motorSpeed){
  // Right HBridge, Right motor drive forward.  
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(_RSTBY, HIGH); // Left HBridge Motors ON 
  analogWrite(_PWMRR, motorSpeed); // Provide PWM Created Analog Voltage to Motors
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation. 
  digitalWrite(_RRMotorDirection1, LOW);     
  digitalWrite(_RRMotorDirection2, HIGH);
}

void PheenoV2Basic::reverseLL(int motorSpeed){
  // Left HBridge, Left motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(_LSTBY, HIGH); // Left HBridge Motors ON 
  analogWrite(_PWMLL, motorSpeed); // Provide PWM Created Analog Voltage to Motors
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation.
  digitalWrite(_LLMotorDirection1, HIGH);    
  digitalWrite(_LLMotorDirection2, LOW);  
}

void PheenoV2Basic::reverseLR(int motorSpeed){
  // Left HBridge, Right motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(_LSTBY, HIGH); // Left HBridge Motors ON 
  analogWrite(_PWMLR, motorSpeed); // Provide PWM Created Analog Voltage to Motors
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation.
  digitalWrite(_LRMotorDirection1, LOW);    
  digitalWrite(_LRMotorDirection2, HIGH);  
}

void PheenoV2Basic::reverseRL(int motorSpeed){
  // Right HBridge, Left motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(_RSTBY, HIGH); // Right HBridge Motors ON 
  analogWrite(_PWMRL, motorSpeed); // Provide PWM Created Analog Voltage to Motors
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation.
  digitalWrite(_RLMotorDirection1, LOW);    
  digitalWrite(_RLMotorDirection2, HIGH);  
}

void PheenoV2Basic::reverseRR(int motorSpeed){
  // Right HBridge, Right Motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(_RSTBY, HIGH); // Right HBridge Motors ON 
  analogWrite(_PWMRR, motorSpeed); // Provide PWM Created Analog Voltage to Motors
  //These digital logic set the direction the current flows through the motor 
  //(Which terminal is positive/negative)! This determines the direction of the motor's
  //rotation.
  digitalWrite(_RRMotorDirection1, HIGH);    
  digitalWrite(_RRMotorDirection2, LOW);  
}
///////////////////////////////////////////////////////////
//Controllers
///////////////////////////////////////////////////////////

void PheenoV2Basic::PIDMotorControlLL(float desVelInput){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;
    //Prefilter
    //a = 0.01/(1/36.07+0.01)
    float a = 0.18;//Slightly tweaked to lower overshoot.

    if (millis() - _PIDMotorsTimeStartLL >= timeStep){
      _desVelLL = (1-a)*_desVelLL+a*desVelInput;
      float desVel = _desVelLL;
      float PIDTimeStep = (millis() - _PIDMotorsTimeStartLL)/1000;//Time step for controller to work on (s).

      readEncoders();
      int countLL = encoderCountLL;

      // Error on individual motors for vel control
      float errorLL = desVel - 2.0 * _pi * (countLL - _oldMotorPIDEncoderCountLL) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      // Check and correct for rollover
      if (countLL < 0 && _oldMotorPIDEncoderCountLL > 0 && _oldMotorPIDEncoderCountLL > 20000){
        errorLL = desVel - 2.0 * _pi * ((countLL - (-32768)) - (32767 - _oldMotorPIDEncoderCountLL)) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      if (countLL > 0 && _oldMotorPIDEncoderCountLL < 0 && _oldMotorPIDEncoderCountLL < -20000){
        errorLL = desVel - 2.0 * _pi * ((32767 - countLL) - (_oldMotorPIDEncoderCountLL - (-32768))) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      _integralLL += errorLL * PIDTimeStep;
      float diffLL = (errorLL - _oldSpeedErrorLL) / PIDTimeStep;
      _oldSpeedErrorLL = errorLL;
      _oldMotorPIDEncoderCountLL = countLL;
      if (_satLL){
        _motorLLVolt += int(_kpMotor*errorLL + _kiMotor*(_integralLL + _satLLVal*PIDTimeStep) + _kdMotor*diffLL);
        _satLL = false;
      }
      else{
        _motorLLVolt += int(_kpMotor*errorLL + _kiMotor*_integralLL + _kdMotor*diffLL);
      }

      if (_motorLLVolt>255){
        _integralLL -= errorLL * PIDTimeStep;
        _satLL=true;
        _satLLVal = (255 - _motorLLVolt);
        _motorLLVolt=255;
      }
      if (_motorLLVolt<-255){
        _integralLL -= errorLL * PIDTimeStep;
        _satLL=true;
        _satLLVal = (-255 - _motorLLVolt);
        _motorLLVolt=-255;
      }
      if (_motorLLVolt >= 0){
        forwardLL(_motorLLVolt);
      }
      if (_motorLLVolt < 0){
        reverseLL(abs(_motorLLVolt));
      }
      _PIDMotorsTimeStartLL = millis();
    }
}

void PheenoV2Basic::PIDMotorControlLR(float desVelInput){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;
    //Prefilter
    //a = 0.01/(1/36.07+0.01)
    float a = 0.18;//Slightly tweaked to lower overshoot.

    if (millis() - _PIDMotorsTimeStartLR >= timeStep){
      _desVelLR = (1-a)*_desVelLR+a*desVelInput;
      float desVel = _desVelLR;
      float PIDTimeStep = (millis() - _PIDMotorsTimeStartLR)/1000.0;//Time step for controller to work on (s).

      readEncoders();
      int countLR = encoderCountLR;

      // Error on individual motors for vel control
      float errorLR = desVel - 2.0 * _pi * (countLR - _oldMotorPIDEncoderCountLR) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      // Check and correct for rollover
      if (countLR < 0 && _oldMotorPIDEncoderCountLR > 0 && _oldMotorPIDEncoderCountLR > 20000){
        errorLR = desVel - 2.0 * _pi * ((countLR - (-32768)) - (32767 - _oldMotorPIDEncoderCountLR)) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      if (countLR > 0 && _oldMotorPIDEncoderCountLR < 0 && _oldMotorPIDEncoderCountLR < -20000){
        errorLR = desVel - 2.0 * _pi * ((32767 - countLR) - (_oldMotorPIDEncoderCountLR - (-32768))) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      _integralLR += errorLR * PIDTimeStep;
      float diffLR = (errorLR - _oldSpeedErrorLR) / PIDTimeStep;
      _oldSpeedErrorLR = errorLR;
      _oldMotorPIDEncoderCountLR = countLR;

      if (_satLR){
        _motorLRVolt += int(_kpMotor*errorLR + _kiMotor*(_integralLR + _satLRVal*PIDTimeStep) + _kdMotor*diffLR);
        _satRL = false;
      }
      else{
        _motorLRVolt += int(_kpMotor*errorLR + _kiMotor*_integralLR + _kdMotor*diffLR);
      }

      if (_motorLRVolt>255){
        _integralLR -= errorLR * PIDTimeStep;
        _satLR=true;
        _satLRVal = (255 - _motorLRVolt);
        _motorLRVolt=255;
      }
      if (_motorLRVolt<-255){
        _integralLR -= errorLR * PIDTimeStep;
        _satLR=true;
        _satLRVal = (-255 - _motorLRVolt);
        _motorLRVolt=-255;
      }
      if (_motorLRVolt >= 0){
        forwardLR(_motorLRVolt);
      }
      if (_motorLRVolt < 0){
        reverseLR(abs(_motorLRVolt));
      }
      _PIDMotorsTimeStartLR = millis();
    }
}

void PheenoV2Basic::PIDMotorControlRL(float desVelInput){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;

    //Prefilter
    //a = 0.01/(1/36.07+0.01)
    float a = 0.18;//Slightly tweaked to lower overshoot.

    if (millis() - _PIDMotorsTimeStartRL >= timeStep){
      _desVelRL = (1-a)*_desVelRL+a*desVelInput;
      float desVel = _desVelRL;

      float PIDTimeStep = (millis() - _PIDMotorsTimeStartRL)/1000.0;//Time step for controller to work on (s).

      readEncoders();
      int countRL = encoderCountRL;

      // Error on individual motors for vel control
      float errorRL = desVel - 2.0 * _pi * (countRL - _oldMotorPIDEncoderCountRL) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      // Check and correct for rollover
      if (countRL < 0 && _oldMotorPIDEncoderCountRL > 0 && _oldMotorPIDEncoderCountRL > 20000){
        errorRL = desVel - 2.0 * _pi * ((countRL - (-32768)) - (32767 - _oldMotorPIDEncoderCountRL)) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      if (countRL > 0 && _oldMotorPIDEncoderCountRL < 0 && _oldMotorPIDEncoderCountRL < -20000){
        errorRL = desVel - 2.0 * _pi * ((32767 - countRL) - (_oldMotorPIDEncoderCountRL - (-32768))) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      _integralRL += errorRL* PIDTimeStep;
      float diffRL = (errorRL - _oldSpeedErrorRL) / PIDTimeStep;
      _oldSpeedErrorRL = errorRL;
      _oldMotorPIDEncoderCountRL = countRL;

      if (_satRL){
        _motorRLVolt += int(_kpMotor*errorRL + _kiMotor*(_integralRL + _satRLVal*PIDTimeStep) + _kdMotor*diffRL);
        _satRL = false;
      }
      else{
        _motorRLVolt += int(_kpMotor*errorRL + _kiMotor*_integralRL + _kdMotor*diffRL);
      }

      if (_motorRLVolt>255){
        _integralRL -= errorRL* PIDTimeStep;
        _satRL = true;
        _satRLVal = 255 - _motorRLVolt;
        _motorRLVolt=255;
      }
      if (_motorRLVolt<-255){
        _integralRL -= errorRL* PIDTimeStep;
        _satRL = true;
        _satRLVal = -255 - _motorRLVolt;
        _motorRLVolt=-255;
      }
      if (_motorRLVolt >= 0){
        forwardRL(_motorRLVolt);
      }
      if (_motorRLVolt < 0){
        reverseRL(abs(_motorRLVolt));
      }
      _PIDMotorsTimeStartRL = millis();
    }
}

void PheenoV2Basic::PIDMotorControlRR(float desVelInput){
    //Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s).

    float timeStep = 10;

    //Prefilter
    //a = 0.01/(1/36.07+0.01)
    float a = 0.18;//Slightly tweaked to lower overshoot.

    if (millis() - _PIDMotorsTimeStartRR >= timeStep){
      _desVelRR = (1-a)*_desVelRR+a*desVelInput;
      float desVel = _desVelRR;

      float PIDTimeStep = (millis() - _PIDMotorsTimeStartRR)/1000.0;//Time step for controller to work on (s).

      readEncoders();
      int countRR = encoderCountRR;

      //Compute Error
      float errorRR = desVel - 2.0 * _pi * (countRR - _oldMotorPIDEncoderCountRR) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);

      // Check and correct for rollover
      if (countRR < 0 && _oldMotorPIDEncoderCountRR > 0 && _oldMotorPIDEncoderCountRR > 20000){
        errorRR = desVel - 2.0 * _pi * ((countRR - (-32768)) - (32767 - _oldMotorPIDEncoderCountRR)) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      if (countRR > 0 && _oldMotorPIDEncoderCountRR < 0 && _oldMotorPIDEncoderCountRR < -20000){
        errorRR = desVel - 2.0 * _pi * ((32767 - countRR) - (_oldMotorPIDEncoderCountRR - (-32768))) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      }
      _integralRR += (errorRR * PIDTimeStep);
      float diffRR = (errorRR - _oldSpeedErrorRR) / PIDTimeStep;
      _oldSpeedErrorRR = errorRR;
      _oldMotorPIDEncoderCountRR = countRR;

      if (_satRR){
        _motorRRVolt += int(_kpMotor*errorRR + _kiMotor*(_integralRR + _satRRVal*PIDTimeStep) + _kdMotor*diffRR);
        _satRR = false;
      }
      else{
        _motorRRVolt += int(_kpMotor*errorRR + _kiMotor*_integralRR + _kdMotor*diffRR);
      }

      if (_motorRRVolt>=255){
        _integralRR -= (errorRR*PIDTimeStep);//Do not integrate at saturation
        _satRR=true;
        _satRRVal = (255 - _motorRRVolt);
        _motorRRVolt=255;
      }
      if (_motorRRVolt<=-255){
        _integralRR -= (errorRR*PIDTimeStep);//Do not integrate at saturation
        _satRR = true;
        _satRRVal = (-255 - _motorRRVolt);
        _motorRRVolt=-255;
      }
      if (_motorRRVolt >= 0){
        forwardRR(_motorRRVolt);
      }
      if (_motorRRVolt < 0){
        reverseRR(abs(_motorRRVolt));
      }


      _PIDMotorsTimeStartRR = millis();
    }
}

///////////////////////////////////////////////////////////
//Observers
///////////////////////////////////////////////////////////

void PheenoV2Basic::encoderPositionUpdate(float timeStep){

  if (millis() - _positionUpdateTimeStart >= timeStep){
    timeStep = (millis() - _positionUpdateTimeStart)/1000; //Convert ms to s
    _positionUpdateTimeStart = millis();

    readEncoders();
    int countL = encoderCountLR;
    int countR = encoderCountRL;

    float Dl = _pi*wheelDiameter * (countL - _oldEPUEncoderCountL) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    float Dr = _pi*wheelDiameter * (countR - _oldEPUEncoderCountR) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    //Check integer roll over!
    if (countL < 0 && _oldEPUEncoderCountL > 0 && _oldEPUEncoderCountL > 20000){
      Dl = _pi*wheelDiameter * ((countL - (-32768)) + (32767 - _oldEPUEncoderCountL)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    if (countR < 0 && _oldEPUEncoderCountR > 0 && _oldEPUEncoderCountL > 20000){
      Dr = _pi*wheelDiameter * ((countR - (-32768)) + (32767 - _oldEPUEncoderCountR)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    float Dc = (Dr + Dl)/2; //Distance center of bot has moved read by encoders.
    _oldEPUEncoderCountR = countR;
    _oldEPUEncoderCountL = countL;
    
    botA += (Dr - Dl)/axelLength;

    botXPos += Dc * cos(_botA0 + (botA-_botA0)/2);
    botYPos += Dc * sin(_botA0 + (botA-_botA0)/2);

    botVel = (Dr + Dl)/(2*timeStep);
    _botA0 = botA;
  }  
}

void PheenoV2Basic::sensorFusionPositionUpdate(float northOffset){

  float timeStep = 10;

  if (millis() - _positionUpdateTimeStart >= timeStep){
    timeStep = (millis() - _positionUpdateTimeStart)/1000; //Convert ms to s
    _positionUpdateTimeStart = millis();

    readEncoders();
    int countL = encoderCountLR;
    int countR = encoderCountRL;

    float Dr = _pi * wheelDiameter * (countR - _oldSFPUEncoderCountR) / (encoderCountsPerRotation * motorGearRatio); // Linear distance right wheel has rotated.
    float Dl = _pi * wheelDiameter * (countL - _oldSFPUEncoderCountL) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    //Check integer roll over!
    if (countL < 0 && _oldSFPUEncoderCountL > 0 && _oldSFPUEncoderCountL > 20000){
      Dl = _pi*wheelDiameter * ((countL - (-32768)) + (32767 - _oldSFPUEncoderCountL)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    if (countR < 0 && _oldEPUEncoderCountR > 0 && _oldSFPUEncoderCountR > 20000){
      Dr = _pi*wheelDiameter * ((countR - (-32768)) + (32767 - _oldSFPUEncoderCountR)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    float Dc = (Dr + Dl)/2; //Distance center of bot has moved read by encoders.
    _oldSFPUEncoderCountR = countR;
    _oldSFPUEncoderCountL = countL;

    float botD = 0.98 * Dc + 0.02 * botVel * timeStep;
    
    readMag(northOffset);
    readGyro();
    if (_botA0 > -_pi/2 && _botA0 < _pi/2){
      botA = 0.9836 * _wrapToPi(botA + IMUGYROZ*timeStep) + 0.0164 * _wrapToPi(IMUHeading);
    }
    else{
      botA =  0.9836 * _wrapTo2Pi(botA + IMUGYROZ*timeStep) + 0.0164 * _wrapTo2Pi(IMUHeading);
    }

    botXPos += botD * cos(_botA0 + (botA-_botA0)/2);
    botYPos += botD * sin(_botA0 + (botA-_botA0)/2);

    readAccel();
    botVel = 0.95 * Dc/timeStep + 0.05 * (botVel + IMUACCY * timeStep);
    _botA0 = botA;
  }  
}