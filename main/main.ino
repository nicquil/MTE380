// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "readGyroscope.h"
#include "sensor_functions.h"
#include "motor_defs.h"
#include <PID_v1.h>


// Adafruit_ICM20948 icm;
// uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
bool TO_RAMP = 0;
bool TURN_TO_RAMP = 0;
bool UP_RAMP = 0;
bool DOWN_RAMP = 0;
bool TURN_FROM_RAMP = 0;
bool BACK_FROM_RAMP = 0;

#define kP 4
#define kI 0.2
#define kD 1

float TOF_TOLERANCE = 100;
double oriX = 0;
double oriY = 0;
double oriZ = 0;

double heading = 0;

unsigned long lastT;

// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Ultra and TOF
  setupUltrasonic();
  setupTOF();

  // Try to initialize Gyro
  setupGyro();
}


void loop() {

}

//Distance detection using ToF sensor
void detectDistance(int distance, int min_detections, int poll_rate){
  int detect_count = 0;
  int bad_detect = 0;
  while (detect_count < min_detections) {
    int measured_dist = readTOF();
    if ((measured_dist < 50) || (measured_dist > 2000)){
      bad_detect++;
      if (bad_detect > 1) {
        detect_count = 0;
        bad_detect = 0;
        Serial.println("");
      }
    }
    else if ((measured_dist > (distance - TOF_TOLERANCE)) && (measured_dist < (distance + TOF_TOLERANCE))) {
        detect_count++;
        Serial.println("count incremented; distance detected");
    }
    delay(poll_rate);
  }
}

//Drive straight using PID control for a certain amount of time
void driveTime(int ms){
  lastT = millis();
  double currL = 220;
  double currR = 220;

  Axes gyroOut;  
  
  PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT);
  
  driveLeft(currL);
  driveRight(currR);
  unsigned long startT = millis();
  lastT = millis();
  myPID.SetMode(AUTOMATIC);

  while((millis() - startT) < ms){
    gyroOut = readGyro(oriX, oriY, oriZ);
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    myPID.Compute();
    driveLeft(currL);
  }
  oriX = 0;
  oriY = 0;
  oriZ = 0;
  stop_motors();
}

//Turn robert by a certain degrees
void turn(int deg) {
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;
  lastT = millis();
  Axes gyroOut;
  if(deg >= 0){
    driveLeft(-230);
    driveRight(230);
    while(oriZ < deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1010; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(230);
    driveRight(-230);
    while(oriZ > deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      lastT = millis();
      // delay(10);
    }
  }
  stop_motors();
  oriX = 0;
  oriY = 0;
  oriZ = 0;
}
