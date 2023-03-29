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
#define kP 0.001 //0.005 //0.001
#define kI 0.1 //0.1
#define kD 0.003 //0.003
double oriX = 0;
double oriY = 0;
double oriZ = 0;
double heading = 0;
unsigned long lastT;

float TOF_TOLERANCE = 25; // 5cm on tolerance for distance detection
bool DIST_DETECTED = 0;
bool TO_RAMP = 1;

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Sensors
  setupUltrasonic();
  setupTOF();
  setupGyro();
}
void loop() {
  
  if (TO_RAMP == 1) {
    // initial move for 2sec
    Serial.println("START MOTORS");    
    driveTime(500, 190, 200);
    driveTime(1500, 160, 160);
    
    // begin detection for distance
    detectDistance(700, 10, 25);

    // once detected, stop motors
    delay(1000);
    stop_motors();
    Serial.println("STOP MOTORS");
    delay(2000);

    // turn 90 to align
    turn(90);  
    // drive forward and check gyro changes 
    driveTime(2000, 170, 170);

    // when gyro changes, flag that the robot is on the ramp
  
    // update global flags for next section of course
    // TO_RAMP = 0;
    // ON_RAMP = 1;
  }
}

void turn(int deg) {
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;
  lastT = millis();
  Axes gyroOut;
  if(deg >= 0){
    driveLeft(-228);
    driveRight(228);
    while(oriZ < deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1010; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(228);
    driveRight(-228);
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

void driveTime(int ms, double currL, double currR){
  lastT = millis();
  double lSpeed = currL;
  Axes gyroOut;  
  
  PID myPID(&oriZ, &lSpeed, &heading, kP, kI, kD, P_ON_M, DIRECT);
  
  driveLeft(currL);
  driveRight(currR);
  unsigned long startT = millis();
  lastT = millis();
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  while((millis() - startT) < ms){
    gyroOut = readGyro(oriX, oriY, oriZ);
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    myPID.Compute();
    driveLeft(lSpeed);    
  }
  oriX = 0;
  oriY = 0;
  oriZ = 0;
  stop_motors();
}
