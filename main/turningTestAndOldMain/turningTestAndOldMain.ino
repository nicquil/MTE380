// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "readGyroscope.h"
#include "sensor_functions.h"
#include "motor_defs.h"


// Adafruit_ICM20948 icm;
// uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
float oriX = 0;
float oriY = 0;
float oriZ = 0;
unsigned long lastT;
// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

void setup(void) {
  setup_serial_i2c();
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Ultra and TOF
  setupUltrasonic();
  setupTOF();
  motor_setup();
  // Try to initialize Gyro
  setupGyro();
}


void loop() {
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;

  Serial.print("Orientation X: ");
  Serial.println(oriX);
  Serial.print("Orientation Y: ");
  Serial.println(oriY);
  Serial.print("Orientation Z: ");
  Serial.println(oriZ);
  Serial.println();

  // readTOF();
  // readUltrasonic(); // may be skipping some ultrasonic sensor reads due to byte reading in function (defined in sensor_functions.cpp)
  
  // test1();
  test2();

  delay(5000);
}

void driveTime(int ms){
  driveLeft(224);
  driveRight(221);
  delay(ms);
  stop_motors();
}

void turn(int deg) {
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;
  lastT = millis();
  Axes gyroOut;
  if(deg >= 0){
    driveLeft(-224);
    driveRight(224);
    while(oriZ < deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/970;
      lastT = millis();
      // delay(10);
    }
    stop_motors();
    oriX = 0;
    oriY = 0;
    oriZ = 0;
  }
  else{
    driveLeft(224);
    driveRight(-224);
    while(oriZ > deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/970;
      lastT = millis();
      // delay(10);
    }
    stop_motors();
    oriX = 0;
    oriY = 0;
    oriZ = 0;
  }
}

void test1(){
    
  driveTime(4000);
  delay(1000);

  turn(90);
  delay(1000);

  driveTime(4000);
  delay(1000);
  turn(90);
  delay(1000);

  driveTime(4000);
  delay(1000);
  turn(90);
  delay(1000);

  driveTime(4000);
  delay(1000);
  turn(90);
  delay(1000);

}

void test2(){
  turn(90);
  delay(1000);
  
  turn(90);
  delay(1000);

  turn(90);
  delay(1000);

  turn(90);
  delay(1000);

  turn(180);
  delay(1000);

  turn(-180);
  delay(1000);

  turn(360);
  delay(1000);

  turn(-360);
  delay(1000);

}