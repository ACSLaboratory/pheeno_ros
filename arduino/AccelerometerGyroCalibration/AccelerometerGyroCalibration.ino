/* This script calibrates and saves the euler angles that define the
 * rotation from the reference frame of the IMU to Pheeno's reference 
 * frame. Also saves the resting gyro bias to correct the gyroscope 
 * measurements.
 * 
 * Sean Wilson, ASU 2017.
 */

#include <Wire.h>
#include <LSM6.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

LSM6 imu;
Adafruit_NeoPixel LED = Adafruit_NeoPixel(8, 13, NEO_GRB + NEO_KHZ800);

//USER INPUT THE KNOWN PITCH ANGLE!
float knownPitch = atan2(0.988,5.088);

int numDataPoints = 6000;//Number of calibration data points to average over.

float count = 1;//Counter to average accelerometer readings effectively.
int eeAddress = 0;//Address to store calibration data.

//Average Accelerometer Values
float accXAvg = 0;
float accYAvg = 0;
float accZAvg = 0;

//Average Gyro Values
float gyroXAvg = 0;
float gyroYAvg = 0;
float gyroZAvg = 0;

//Euler Angles to Store
float storePitch = 0;
float storeRoll = 0;
float storeYaw = 0;

void setup() {
  //Set up IMU to take readings.
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  LED.begin();
  LED.show(); // Initialize all pixels to 'off'

  //Show the bot is ready to go!
  colorWipe(LED.Color(0, 0, 255),100);
  colorWipe(LED.Color(255, 0, 0),50);
}

void loop() {
  for(int i = 0; i<numDataPoints; i++){
    imu.readAcc();
    //Loop to average accelerometer readings
    accXAvg = (count - 1.0)/count * accXAvg + 1.0/count * imu.a.x*0.0059841;
    accYAvg = (count - 1.0)/count * accYAvg + 1.0/count * -imu.a.y*0.0059841;//Negative Value is here for RH coordinate frame.
    accZAvg = (count - 1.0)/count * accZAvg + 1.0/count * imu.a.z*0.0059841;

    imu.readGyro();
    //Loop to average accelerometer readings
    gyroXAvg = (count - 1.0)/count * gyroXAvg + 1.0/count * imu.g.x;
    gyroYAvg = (count - 1.0)/count * gyroYAvg + 1.0/count * imu.g.y;
    gyroZAvg = (count - 1.0)/count * gyroZAvg + 1.0/count * imu.g.z;
    
    
    count = count + 1;
    delay(10);   
  }

  //Deterime roll and pitch angles.
  storePitch = atan2(-accYAvg,-accZAvg);
  storeRoll = atan2(accXAvg,sqrt(pow(accYAvg,2)+pow(accZAvg,2)));

  //User needs to pitch at known angle. Give them 15 seconds to do so.
  //Flash lights to let user know robot is ready to pitch.
  float startTime = millis();
  while (millis() - startTime <= 15000){
    colorWipe(LED.Color(0,255,0),50);
    colorWipe(LED.Color(0,0,0),50);
  }
  colorWipe(LED.Color(255,0,0),50);
  //Reset the accelerometer averages.
  accXAvg = 0;
  accYAvg = 0;
  accZAvg = 0;

  count = 1; //Resest count for new average
  
  for(int j = 0; j<numDataPoints; j++){
    imu.readAcc();
    //Loop to average accelerometer readings
    accXAvg = (count - 1)/count * accXAvg + 1/count * imu.a.x*0.0059841;
    accYAvg = (count - 1)/count * accYAvg + 1/count * -imu.a.y*0.0059841;//Negative Value is here for RH coordinate frame.
    accZAvg = (count - 1)/count * accZAvg + 1/count * imu.a.z*0.0059841;
    
    count = count + 1;
    delay(10);   
  }
  
  //Determine Yaw

  //Do Roll and Pitch Correction Calculated Previously.
  // Rx(pitch)Ry(roll)Rz(0) = [cos(roll), 0, -sin(roll);
  //                          sin(roll)sin(pitch), cos(pitch), cos(roll)sin(pitch);
  //                          cos(pitch)sin(roll),-sin(pitch),cos(roll)cos(pitch)]

  float vx = -sin(storeRoll)*-cos(knownPitch);
  float vy = cos(storePitch)*-sin(knownPitch) + cos(storeRoll)*sin(storePitch)*-cos(knownPitch);
  
  storeYaw = asin(((accXAvg/sqrt(pow(accXAvg,2)+pow(accYAvg,2)+pow(accZAvg,2)))*vy \
             -(accYAvg/sqrt(pow(accXAvg,2) + pow(accYAvg,2)+pow(accZAvg,2)))*vx) \
             /(pow(vx,2) + pow(vy,2)));

  //Store Values to the EEPROM
  EEPROM.put(eeAddress, -storePitch);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, -storeRoll);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, -storeYaw);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, int(gyroXAvg));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(gyroYAvg));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(gyroZAvg));
  
  //Blue Lights Mean Bot is Done Calibrating!
  while(1){
    colorWipe(LED.Color(0,0,255),50);
    colorWipe(LED.Color(0,0,0),50);
  }
  
  
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<LED.numPixels(); i++) {
    LED.setPixelColor(i, c);
    LED.show();
    delay(wait);
  }

  
}
