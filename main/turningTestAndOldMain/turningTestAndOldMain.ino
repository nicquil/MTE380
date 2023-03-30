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

int currCase = 1;

double oriX = 0;
double oriY = 0;
double oriZ = 0;

double currL = 210;
double currR = 210;

double heading = 0;

bool change = true;

double lastT = millis();

PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT); //new: DIRECT


// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

void setup(void) {
  setup_serial_i2c();
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Ultra and TOF
  // setupUltrasonic();
  setupTOF();
  motor_setup();
  // Try to initialize Gyro
  setupGyro();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
}


void loop() {
  //---------------------testing pid w/ turn----------------
  // if (change){
  //   lastT = millis();
  //   change = false;
  // }
  // driveTime(currL, currR);
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriZ += gyroOut.z*(millis()-lastT)/1010;

  // myPID.Compute();
  // if (millis() - lastT > 3000){
  //   oriZ = 0;
  //   turn(180);
  //   lastT = millis();
  // } // ^^^  works

  // driveLeft(210);
  // driveRight(210);
//---------------------------------------GOOD FIND POLE CODE VVVV----------------------
  // driveTime(150, 150);
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX += gyroOut.x*(millis()-lastT)/1010;
  // oriY += gyroOut.y*(millis()-lastT)/1010;
  
  // lastT = millis();
  // Serial.println(oriX);
  // if((oriX <= -2) || (abs(oriY) > 3)){
  //   // driveTime(230, 230);
  //   // delay(3000);
  //   stop_motors();
  //   oriX = 0;
  //   oriY = 0;
  //   delay(3000);
  //   lastT = millis();
  

//---------------------------------------GOOD FIND POLE CODE ^^^^^^^^^^^^^^----------------------


  // driveTime(230, 230);
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriY += gyroOut.Y*(millis()-lastT)/1010;
  // if(oriY > 0){
  //   stop_motors();
  //   delay(1000);
  //   turn(90);
  //   oriY = 0;
  // }

//--------game day switch case---------------------------------------------
  //   case 0: //find ramp

  //     driveTime(currL, currR);
  //     Axes gyroOut = readGyro(oriX, oriY, oriZ);
  //     oriZ += gyroOut.z*(millis()-lastT)/1010;

  //     myPID.Compute();
      
  //     if(rampDetected){ //if tof finds ramp
  //       stop_motors();
  //       turn(90);
  //       currCase++;
  //       stop_motors();
  //       break;
  //     }

  if(currCase == 1){ //ascend
    driveTime(150, 150);
    // gyroOut = readGyro(oriX, oriY, oriZ);
    // oriZ += gyroOut.z*(millis()-lastT)/1010;

    // myPID.Compute();
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriX += gyroOut.x*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriX >= 10){ //if ultra finds top
      currCase++;
      driveTime(200, 200);
      lastT = millis();
      oriX = 0;
    }
  }
  else if (currCase == 2){ //top of ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriX += gyroOut.x*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriX >= 10){ //if ultra finds ramp down
      driveTime(220, 220);
      currCase++;
      lastT = millis();
      // delay(2000);
      oriX = 0;
    }
  }
  else if (currCase == 3){ //descend
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriX += gyroOut.x*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriX >= 10){ //if gyro reads 0 on y (yaw? idk lmao)
      // turn(90);
      driveTime(240, 240);
      currCase++;
      lastT = millis();

      currCase++;
      // currL = 150;
      // currR = 150;
      oriX = 0;
    }
  }

  else if (currCase == 4){ //get off ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriX += gyroOut.x*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriX <= -10){ //if gyro finds flat ground
      stop_motors();
      delay(1000);
      turn(90);
      // currL = 150;
      // currR = 150;
      currCase++;
      stop_motors();
    }
  }
}
  // case 5: //go to pole
//     driveTime(currL, currR);
//     gyroOut = readGyro(oriX, oriY, oriZ);
//     oriZ += gyroOut.z*(millis()-lastT)/1010;

//     myPID.Compute();

//     if(onBase){ //if gyro reads > 0 on y (needs tolerance)
//       stop_motors();
//       break;
//     }
// }
// delay(10);

void driveTime(double currL, double currR){
  // double currL = 220; //new: 220 old: 55
  // double currR = 220; //new: 220 old: 52
  // Axes gyroOut;  
  
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
    driveLeft(-225); //new: 70 old: -60
    driveRight(225); //new: -70 old: 52
    while(oriZ < (deg*0.96)){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1000; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(-225); //new: 70 old: 60
    driveRight(225); //new: -70 old: -52
    while(oriZ > (deg*0.96)){
      gyroOut = readGyro(oriX, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1000;
      lastT = millis();
      // delay(10);
    }
  }
  stop_motors();
  oriX = 0;
  oriY = 0;
  oriZ = 0;
}