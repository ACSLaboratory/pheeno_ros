/* This script calibrates and saves the hard iron and soft iron bias corrections
 *  for the magnetometer in the EEPROM.
 * 
 * Sean Wilson, ASU 2017.
 */

#include <Wire.h>
#include <LIS3MDL.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
Adafruit_NeoPixel LED = Adafruit_NeoPixel(8, 13, NEO_GRB + NEO_KHZ800);

int eeAddress = 0;

float calibrationTime = 60000;//How long to calibrate for (ms)
float timeStart = 0;//Start time of calibration set in Setup()

int HIBx = 0;//Hard Iron Bias X
int HIBy = 0;//Hard Iron Bias Y
int HIBz = 0;//Hard Iron Bias Z

float SIBx = 0;//Soft Iron Bias X
float SIBy = 0;//Soft Iron Bias Y
float SIBz = 0;//Soft Iron Bias Z

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
  
  LED.begin();
  LED.show(); // Initialize all pixels to 'off'

  //Show the bot is ready to go!
  colorWipe(LED.Color(0, 0, 255),500);
  timeStart = millis();//Start the calibration period.
}

void loop()
{
  //Collect Data while User Rotates Robot.
  while(millis() - timeStart <= calibrationTime){
    mag.read();
  
    running_min.x = min(running_min.x, mag.m.x);
    running_min.y = min(running_min.y, mag.m.y);
    running_min.z = min(running_min.z, mag.m.z);
  
    running_max.x = max(running_max.x, mag.m.x);
    running_max.y = max(running_max.y, mag.m.y);
    running_max.z = max(running_max.z, mag.m.z);
    colorWipe(LED.Color(0,255,0),50);
    colorWipe(LED.Color(0,0,0),50);
  }

  //Calculate Hard Iron Bias from Data
  HIBx = (running_max.x + running_min.x)/2;
  HIBy = (running_max.y + running_min.y)/2;
  HIBz = (running_max.z + running_min.z)/2;

  //Calculate Soft Iron Bias from Data
  float MCx = (running_max.x  - running_min.x)/2;
  float MCy = (running_max.y  - running_min.y)/2;
  float MCz = (running_max.z  - running_min.z)/2;

  float avg_rad = (MCx + MCy + MCz)/3;

  SIBx = avg_rad/MCx;
  SIBy = avg_rad/MCy;
  SIBz = avg_rad/MCz;

  //Store Values to the EEPROM (First few skips are to make sure the
  //accelerometer and gyro calibration numbers are not overwritten.
  eeAddress += sizeof(float);
  eeAddress += sizeof(float);
  eeAddress += sizeof(float);
  eeAddress += sizeof(int);
  eeAddress += sizeof(int);
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(HIBx));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(HIBy));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(HIBz));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, SIBx);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, SIBy);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, SIBz);
  
  //Red Lights Mean Bot is Done Calibrating!
  while(1){
    colorWipe(LED.Color(255,0,0),50);
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
