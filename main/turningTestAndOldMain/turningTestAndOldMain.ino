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

int currCase = 0;

double oriX = 0;
double oriY = 0;
double oriZ = 0;

double currL = 150;
double currR = 150;

double heading = 0;

PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT); //new: DIRECT


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

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
}


void loop() {
  double lastT = millis();
  switch(currCase){

    case 0: //find ramp

      driveTime(currL, currR);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;

      myPID.Compute();
      
      if(rampDetected){ //if tof finds ramp
        stop_motors();
        turn(90);
        currCase++;
        stop_motors();
        break;
      }
    case 1: //ascend
      driveTime(20, 20);
      // gyroOut = readGyro(oriX, oriY, oriZ);
      // oriZ += gyroOut.z*(millis()-lastT)/1010;

      // myPID.Compute();

      if(topDetected){ //if ultra finds top
        currCase++;
        driveTime(200, 200);
        break;
      }
    case 2: //top of ramp
      if(rampDownDetected){ //if ultra finds ramp down
        driveTime(250, 250);
        currCase++;
        break;
      }
    case 3: //descend
      if(endRampDetected){ //if gyro reads 0 on y (yaw? idk lmao)
        turn(90);
        currCase++;
        currL = 150;
        currR = 150;
        break;
      }
    case 4: //find pole
      driveTime(currL, currR);
      gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;

      myPID.Compute();

      if(poleDetected){ //if tof finds pole
        turn(90);
        currL = 150;
        currR = 150;
        currCase++;
        break;
      }
    case 5: //go to pole
      driveTime(currL, currR);
      gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;

      myPID.Compute();

      if(onBase){ //if gyro reads > 0 on y (needs tolerance)
        stop_motors();
        break;
      }
  }
  delay(5000);
}

void driveTime(double currL, double currR){
  // double currL = 220; //new: 220 old: 55
  // double currR = 220; //new: 220 old: 52
  Axes gyroOut;  
  
  driveLeft(currL);
  driveRight(currR);
}

void turn(int deg) {
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;
  double lastT = millis();
  Axes gyroOut;
  if(deg >= 0){
    driveLeft(-70); //new: 70 old: -60
    driveRight(70); //new: -70 old: 52
    while(oriZ < deg){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1010; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(-70); //new: 70 old: 60
    driveRight(70); //new: -70 old: -52
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