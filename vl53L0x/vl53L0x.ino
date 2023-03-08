/*!
 * @file vl53l0.ino
 * @brief DFRobot's Laser rangefinder library. The example shows the usage of VL53L0X in a simple way.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [LiXin](xin.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-8-21
 * @url https://github.com/DFRobot/DFRobot_VL53L0X
 */
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
//#include "DFRobot_VL53L0X.cpp"

DFRobot_VL53L0X sensor;


void setup() {
  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  
  //setMode with eContinuous / eSingle and eLow / eHigh (Single Reading not useful for our application)
  //Continuous Reading
  //Case 1: Set to Back-to-back mode and high precision mode (.25/.50/.75 mm accuracy -> unnecessary for our application)
  //sensor.setMode(sensor.eContinuous,sensor.eHigh);
  //Case 1: Set to Back-to-back mode and low precision mode (nearest mm ex. 8.8cm -> reports 88.00mm)
  sensor.setMode(sensor.eContinuous,sensor.eLow);


  //Laser rangefinder begins to work
  sensor.start();

  //Announce start of testing and delay 2s  
  Serial.print("Begin Test for Sensor Detection: \n");
  delay(2000);
}

void loop() 
{
  //Get the distance (in mm) every 2s
  Serial.print("Distance: ");Serial.println(sensor.getDistance());
  //The delay is added to demonstrate the effect, and if you do not add the delay,
  //it will not affect the measurement accuracy
  delay(200);
}