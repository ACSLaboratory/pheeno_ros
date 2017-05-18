/* This script constantly reads Pheeno's onboard sensors and prints their results through the
serial port.
*/


#include <PheenoV2Basic.h> // If you want to use Pheeno's premade routines import these libraries!!

PheenoV2Basic myRobot(1); // The argument is the type of robot
                           
float delayTime = 500; //A delay time to make manually reading the serial port easier!

void setup(){
  Serial.begin(9600); //Starts the serial port, with the argument being the bit rate.
  myRobot.SetupBasic(); //This must be included in every script! It sets up all the pins on the robot!
  Serial.println("A test read of all Pheeno's sensors!");
}

void loop() {
  myRobot.readIR(); //Reads all of the IR sensor distances in cm.
  myRobot.readEncoders(); //Reads how many total ticks have occured on the encoders.
  myRobot.readMag(0); //Reads the compass. The argument is the offset between your global x-axis and magnetic north.
  myRobot.readGyro(); //Reads the gyroscope.
  myRobot.readAccel(); //Reads the accelerometer.  
  
  Serial.println("IR Sensor Distances!");
  Serial.println("[Left IR, Left Forward IR, Center IR, Right Forward IR, Right IR, Back IR]");
  Serial.print("[");
  Serial.print(myRobot.LDistance);
  Serial.print(", ");
  Serial.print(myRobot.LFDistance);
  Serial.print(", ");
  Serial.print(myRobot.CDistance);
  Serial.print(", ");
  Serial.print(myRobot.RFDistance);
  Serial.print(", ");
  Serial.print(myRobot.RDistance);
  Serial.print(", ");
  Serial.print(myRobot.BDistance);
  Serial.println("]");
  Serial.println("Encoder Counts!");
  Serial.println("[Left Encoder, Right Encoder]");
  Serial.print("[");
  Serial.print(myRobot.encoderCountLR);
  Serial.print(", ");
  Serial.print(myRobot.encoderCountRL);
  Serial.println("]");
  Serial.println("IMU Measurements (Accelerometer Magnetometer and Gyroscope)!");
  Serial.println("[X Acceleration, Y Acceleration, Z Acceleration, Heading]");
  Serial.print("[");
  Serial.print(myRobot.IMUACCX);
  Serial.print(", ");
  Serial.print(myRobot.IMUACCY);
  Serial.print(", ");
  Serial.print(myRobot.IMUACCZ);
  Serial.print(", ");
  Serial.print(myRobot.IMUHeading);
  Serial.println("]");
  Serial.println("[X Rotational Rate, Y Rotational Rate, Z Rotational Rate]");
    Serial.print("[");
  Serial.print(myRobot.IMUGYROX);
  Serial.print(", ");
  Serial.print(myRobot.IMUGYROY);
  Serial.print(", ");
  Serial.print(myRobot.IMUGYROZ);
  Serial.println("]");
  Serial.println();
  Serial.println();
  
  delay(delayTime); //A simple delay to make manual reading of the serail port easier.
}
