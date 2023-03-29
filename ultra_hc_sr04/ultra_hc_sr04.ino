/**
   HC-SR04 Demo
   Demonstration of the HC-SR04 Ultrasonic Sensor
   Date: August 3, 2016
   Description:
    Connect the ultrasonic sensor to the Arduino as per the
    hardware connections below. Run the sketch and open a serial
    monitor. The distance read from the sensor will be displayed
    in centimeters and inches.
   Hardware Connections:
    Arduino | HC-SR04
    -------------------
      5V    |   VCC
      7     |   Trig
      8     |   Echo
      GND   |   GND
   License:
    Public Domain
*/
#include <Arduino.h>
#include <Wire.h>
#include "motor_defs.h"
// Pins
const int TRIG_PIN = 19;
const int ECHO_PIN = 18;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

void setup() {
  motor_setup();

  // We'll use the serial monitor to view the sensor output
  Serial.begin(9600);
}

void loop() {
  move(200,220,1);
  delay(1000);
  if (readUltra() < 50) {
    driveLeft(200);
    driveRight(-220);
    delay(1500);
    stop_motors();
    delay(3000);
  }
}