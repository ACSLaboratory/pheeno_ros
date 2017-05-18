#ifndef PheenoV2Basic_h
#define PheenoV2Basic_h

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"
#include "EEPROM.h"
#include "Encoder.h"
#include "LSM6.h"
#include "LIS3MDL.h"
#include "Wire.h"
#include "math.h"


class PheenoV2Basic
{
  public:
    PheenoV2Basic(int type);

    ///////////////////////////////////////////////////////////
    //NeoPixel Things
    ///////////////////////////////////////////////////////////

    Adafruit_NeoPixel LED;

    ///////////////////////////////////////////////////////////
    //IR Sensor Distance Variables
    ///////////////////////////////////////////////////////////

    float RDistance;
    float CDistance;
    float LDistance;
    float LFDistance;
    float RFDistance;
    float BDistance;

    ///////////////////////////////////////////////////////////
    //Encoder Counter Variables
    ///////////////////////////////////////////////////////////

    int encoderCountLL; //Left HBridge, Left Motor Encoder Count
    int encoderCountLR; //Left HBridge, Right Motor Encoder Count
    int encoderCountRL; //Right HBridge, Left Motor Encoder Count
    int encoderCountRR; //Right HBridge, Right Motor Encoder Count

    ///////////////////////////////////////////////////////////
    //IMU Variables (Magnetometer/Accelerometer/Gyroscope)
    ///////////////////////////////////////////////////////////

    LSM6 imu; // Contains Gyro and Accelerometer Data
    LIS3MDL mag; // Contains Magnetometer Data

    float IMUACCX; //Acceleration reading in X Direction (cm/s^2)
    float IMUACCY; //Acceleration reading in Y Direction (cm/s^2)
    float IMUACCZ; //Acceleration reading in Z Direction (cm/s^2)

    float IMUMAGX; //Magnetometer reading in X Direction (gauss)
    float IMUMAGY; //Magnetometer reading in Y Direction (gauss)
    float IMUMAGZ; //Magnetometer reading in Z Direction (gauss)

    float IMUGYROX; //Gyroscope reading about X Direction (rad/s)
    float IMUGYROY; //Gyroscope reading about Y Direction (rad/s)
    float IMUGYROZ; //Gyroscope reading about Z Direction (rad/s)

    float IMUHeading; //The direction the robot is facing (yaw angle in rad)
   
    ///////////////////////////////////////////////////////////
    //Constants
    ///////////////////////////////////////////////////////////

    float encoderCountsPerRotation; // Encoder counts per shaft rotation.
    float motorGearRatio; // The gearing ratio of the drive motor being used.
    float wheelDiameter; // Wheel Diameter in cm.
    float axelLength; // Axel length in cm.

    ///////////////////////////////////////////////////////////
    //Robot State Variables
    ///////////////////////////////////////////////////////////

    float botVel;
    float botXPos;
    float botYPos;
    float botA;

    ///////////////////////////////////////////////////////////
    //Functions
    ///////////////////////////////////////////////////////////
    //Setup
    void SetupBasic(); //Sets Pins To Correct Output/Input

    //Sensors
    void readIR(); //Read The IR Sensors
    void readEncoders(); //Read The Encoders
    void readMag(float magNorthOffset);//Read the Magnetometer (In Radians)
    void readAccel();//Read the Accelerometer (in cm/s^2)
    void readGyro();//Read the Gyroscope (in rad/s)

    //Individual Motor Commands
    void forwardLL(int motorVolt); //Rotate Left HBridge, Left Motor Forward using PWM created Voltage. int (0-255)
    void forwardLR(int motorVolt); //Rotate Left HBridge, Right Motor Forward using PWM created Voltage. int (0-255)
    void forwardRL(int motorVolt); //Rotate Right HBridge, Left Motor Forward using PWM created Voltage. int (0-255)
    void forwardRR(int motorVolt); //Rotate Right HBridge, Right Motor Forward using PWM created Voltage. int (0-255)
    
    void reverseLL(int motorVolt); //Rotate Left HBridge, Left Motor Backwards using PWM created Voltage. int (0-255)
    void reverseLR(int motorVolt); //Rotate Left HBridge, Right Motor Backwards using PWM created Voltage. int (0-255)
    void reverseRL(int motorVolt); //Rotate Right HBridge, Left Motor Backwards using PWM created Voltage. int (0-255)
    void reverseRR(int motorVolt); //Rotate Right HBridge, Right Motor Backwards using PWM created Voltage. int (0-255)
    
    //Motor Controllers
    void PIDMotorControlLL(float desVelInput); //Controlled Rotational Shaft Velocity of Left Motor on Left Hbridge (rad/s)
    void PIDMotorControlLR(float desVelInput); //Controlled Rotational Shaft Velocity of Right Motor on Left Hbridge (rad/s)
    void PIDMotorControlRL(float desVelInput); //Controlled Rotational Shaft Velocity of Left Motor on Right Hbridge (rad/s)
    void PIDMotorControlRR(float desVelInput); //Controlled Rotational Shaft Velocity of Right Motor on Right Hbridge (rad/s)

    //Observers
    void sensorFusionPositionUpdate(float northOffset);
    void encoderPositionUpdate(float timeStep);

    //Collective motor commands
    void brakeAll();//Short the Motors to Brake (No Passive Rotation)
    void noMotionAll();//Turn off motors (Allow Passive Rotation)

  private: //Things only accessable within the class (not the user)

    float _deg2rad(float deg); //Converts degrees to radians.
    float _rad2deg(float rad); //Converts radians to degrees.
    float _wrapToPi(float rad); //Wraps a radian angle to (-pi,pi]
    float _wrapTo2Pi(float rad); //Wrap a radian angle to [0,2*pi)

    void _calibrationAccRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz); //Rotates accelerometer data according to accelerometer calibration angles.
    void _calibrationGyroRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz); //Rotates gyroscope data according to accelerometer calibration angles.
    void _calibrationMagRotation(float pitch, float roll, float yaw, float Mx, float My, float Mz); //Rotates magnetometer data according to accelerometer calibration angles.

    ///////////////////////////////////////////////////////////
    //Constants
    ///////////////////////////////////////////////////////////
    static constexpr float _pi = 3.1415926; // Pi...the number...DUH!!
    
    //Constants from distance conversion see getDistance()
    static constexpr double _a = 15.68;// 15.68 
    static constexpr double _b = -0.03907;// -0.03907


    ///////////////////////////////////////////////////////////
    //Pin Numbers Here
    ///////////////////////////////////////////////////////////

    //IR Sensor Input Pins (Analog)
    static constexpr uint8_t _IRC = A18; // A18 // Analog input center sensor
    static constexpr uint8_t _IRLF = A13;// A13 // Analog input left forward sensor
    static constexpr uint8_t _IRL = A10;// A10 // Analog input left sensor
    static constexpr uint8_t _IRB = A11;// A11 // Analog input back sensor
    static constexpr uint8_t _IRRF = A17;// A17 // Analog input right forward sensor
    static constexpr uint8_t _IRR = A12;// A12 // Analog input right sensor

    //Battery Voltage Input Pin
    static constexpr uint8_t _BattVolt = 4; // D4 // Battery Voltage Level

    //Camera Servo Pin
    static constexpr uint8_t _CameraServo = 12;// D12 // Camera servo

    //Daisy Chain LED Pin (NeoPixels)
    static constexpr uint8_t _LED = 13; // D13 // LED (NeoPixel) Strip control

    //Motor Control Output Pins
    static constexpr uint8_t _PWMLL = 25;// D25 // Left HBridge, Left Motor PWM Control
    static constexpr uint8_t _PWMLR = 32;// D32 // Left HBridge, Right Motor PWM Control
    static constexpr uint8_t _PWMRL = 5;// D5 // Right HBridge, Left Motor PWM Control
    static constexpr uint8_t _PWMRR = 6;// D6 // Left HBridge, Left Motor PWM Control
        
    static constexpr uint8_t _LLMotorDirection1 = 27;// D27 // Left HBridge, Left Motor Direction 1
    static constexpr uint8_t _LLMotorDirection2 = 26;// D26 // Left HBridge, Left Motor Direction 2

    static constexpr uint8_t _LRMotorDirection1 = 31;// D31 // Left HBridge, Right Motor Direction 1
    static constexpr uint8_t _LRMotorDirection2 = 33;// D33 // Left HBridge, Right Motor Direction 2
    
    static constexpr uint8_t _RLMotorDirection1 = 8;// D8 // Right HBridge, Left Motor Direction 1
    static constexpr uint8_t _RLMotorDirection2 = 7;// D7 // Right HBridge, Left Motor Direction 2
    
    static constexpr uint8_t _RRMotorDirection1 = 10;// D10 // Right HBridge, Right Motor Direction 1
    static constexpr uint8_t _RRMotorDirection2 = 11;// D11 // Right HBridge, Right Motor Direction 2

    static constexpr uint8_t _LSTBY = 30;// D30 // Standby pin to turn off motors attached to Left HBridge.
    static constexpr uint8_t _RSTBY = 9;// D9 // Standby pin to turn off motors attached to Right HBridge.

    //Motor Feedback Pins (Encoders)
    static constexpr uint8_t _encoder1LL = 17;// D17 // Left HBridge, Left Motor Encoder Interrupt 1
    static constexpr uint8_t _encoder2LL = 16;// D16 // Left HBridge, Left Motor Encoder Interrupt 2

    static constexpr uint8_t _encoder1LR = 15;// D15 // Left HBridge, Right Motor Encoder Interrupt 1
    static constexpr uint8_t _encoder2LR = 14;// D14 // Left HBridge, Right Motor Encoder Interrupt 2

    static constexpr uint8_t _encoder1RL = 1;// D27 // Right HBridge, Left Motor Encoder Interrupt 1
    static constexpr uint8_t _encoder2RL = 0;// D26 // Right HBridge, Left Motor Encoder Interrupt 2

    static constexpr uint8_t _encoder1RR = 3;// D2 // Right HBridge, Right Motor Encoder Interrupt 1
    static constexpr uint8_t _encoder2RR = 2;// D2 // Right HBridge, Right Motor Encoder Interrupt 2    

    ///////////////////////////////////////////////////////////
    //Motor PID Control Constants
    ///////////////////////////////////////////////////////////

    //Motor Controller Timers
    float _PIDMotorsTimeStartLL;
    float _PIDMotorsTimeStartLR;
    float _PIDMotorsTimeStartRL;
    float _PIDMotorsTimeStartRR;

    //Integral Error Holders
    float _integralLL;
    float _integralLR;
    float _integralRL;
    float _integralRR;

    float _kpMotor;// Continuous Proportional Gain
    float _kiMotor;// Continuous Integral Gain
    float _kdMotor;// Continuous Derivative Gain

    ///////////////////////////////////////////////////////////
    //IMU Things (Gyro/Accelerometer/Magnetometer)
    ///////////////////////////////////////////////////////////

    int _eeAddress = 0;// Address to store calibration data in EEPROM

    float _accPitchCalibration = 0;// Accelerometer Pitch rotation (rad) from IMU to Robot Frame (roll->pitch->yaw rotation)
    float _accRollCalibration = 0;// Accelerometer Roll rotation (rad) from IMU to Robot Frame (roll->pitch->yaw rotation)
    float _accYawCalibration= 0;// Accelerometer Yaw rotation (rad) from IMU to Robot Frame (roll->pitch->yaw rotation)

    int _gyroOffX= 0;// Gyroscope offset about the X-Direction when resting.
    int _gyroOffY = 0;// Gyroscope offset about the Y-Direction when resting.
    int _gyroOffZ= 0;// Gyroscope offset about the Z-Direction when resting.

    int _magOffX= 0;// Magnetometer Hard Iron offset in the X-Direction.
    int _magOffY = 0;// Magnetometer Hard Iron offset in the Y-Direction.
    int _magOffZ= 0;// Magnetometer Hard Iron offset in the Z-Direction.

    float _magScaleX= 0;// Magnetometer Soft Iron bias approximation in the X-Direction.
    float _magScaleY = 0;// Magnetometer Soft Iron bias approximation in the Y-Direction.
    float _magScaleZ= 0;// Magnetometer Soft Iron bias approximation in the Z-Direction.

    ///////////////////////////////////////////////////////////
    //Encoder Things
    ///////////////////////////////////////////////////////////

    Encoder _motorLL, _motorLR, _motorRL, _motorRR;

    int _motorLLVolt;//Left HBridge, Left Motor Motor Input Voltage (Arduino PWM Units, int 0-255)
    int _motorLRVolt;//Left HBridge, Right Motor Motor Input Voltage (Arduino PWM Units, int 0-255)
    int _motorRLVolt;//Right HBridge, Left Motor Motor Input Voltage (Arduino PWM Units, int 0-255)
    int _motorRRVolt;//Right HBridge, Right Motor Motor Input Voltage (Arduino PWM Units, int 0-255)

    float _oldSpeedErrorLL; //Left HBridge, Left Motor old rotation speed error for PIDMotorControl Function
    float _oldSpeedErrorLR; //Left HBridge, Right Motor old rotation speed error for PIDMotorControl Function
    float _oldSpeedErrorRL; //Right HBridge, Left Motor old rotation speed error for PIDMotorControl Function
    float _oldSpeedErrorRR; //Right HBridge, Right Motor old rotation speed error for PIDMotorControl Function

    int _oldMotorPIDEncoderCountLL;//Left HBridge, Left Motor old Encoder Count storage for PIDMotorControl Function
    int _oldMotorPIDEncoderCountLR;//Left HBridge, Right Motor old Encoder Count storage for PIDMotorControl Function
    int _oldMotorPIDEncoderCountRL;//Right HBridge, Left Motor old Encoder Count storage for PIDMotorControl Function
    int _oldMotorPIDEncoderCountRR;//Right HBridge, Right Motor old Encoder Count storage for PIDMotorControl Function

    float _desVelRR;
    float _desVelRL;
    float _desVelLR;
    float _desVelLL;

    bool _satRR;
    int _satRRVal;
    bool _satRL;
    int _satRLVal;
    bool _satLR;
    int _satLRVal;
    bool _satLL;
    int _satLLVal;

    ///////////////////////////////////////////////////////////
    //Observer Things
    ///////////////////////////////////////////////////////////
    float _positionUpdateTimeStart;

    int _oldEPUEncoderCountL;//Old Encoder Count storage for encoderPositionUpdate Function
    int _oldEPUEncoderCountR;

    int _oldSFPUEncoderCountL;//Old Encoder Count storage for sensorFusionPositionUpdate Function
    int _oldSFPUEncoderCountR;  

    float _botA0;//Storage for last heading of robot used in sensorFusionPositionUpdate  
};

#endif
