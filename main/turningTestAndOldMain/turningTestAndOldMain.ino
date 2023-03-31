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
int currCount = 0;

double oriX = 0;
double oriY = 0;
double oriZ = 0;

double currL = 210;
double currR = 210;
double heading = 0;

double lastT = millis();

PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT); //new: DIRECT


// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

void setup(void) {
  setup_serial_i2c();
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Ultra and TOF
  setupUltra();
  // setupTOF();
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
  // Axes gyroOut = readGyro(oriY, oriY, oriZ);
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
  // Axes gyroOut = readGyro(oriY, oriY, oriZ);
  // oriY += gyroOut.x*(millis()-lastT)/1010;
  // oriY += gyroOut.y*(millis()-lastT)/1010;
  
  // lastT = millis();
  // Serial.println(oriY);
  // if((oriY <= -2) || (abs(oriY) > 3)){
  //   // driveTime(230, 230);
  //   // delay(3000);
  //   stop_motors();
  //   oriY = 0;
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

  // driveTime(0, 0);
// --------game day switch case---------------------------------------------
  // driveTime(currL, currR)
    
    if (currCase == 0){ //find ramp
      driveTime(200, 195);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      myPID.Compute();
      lastT = millis();

      float measured_dist = readUltra();
    
      if (measured_dist <= 40){
        currCount += 1;
      }
      else{
        currCount = 0;
      }
      if (currCount == 3){
        delay(10);
        turn(-90);
        stop_motors();
        delay(1000);
        currCase++;   
        oriZ = 0;     
        oriY = 0;
        lastT = millis();
      }
      // driveTime(currL, currR);
//------------------wiggle code----------------------
      // Axes gyroOut;
      // gyroOut = readGyro(oriX, oriY, oriZ);
      // oriY += gyroOut.y*(millis()-lastT)/1010;
      
      // lastT = millis();

      // driveLeft(255);
      // driveRight(55);
      // delay(300);


      // driveLeft(55);
      // driveRight(255);
      // delay(300);
      
      // if(oriY <= -10){ //if tof finds ramp
      //   stop_motors();
      //   delay(1000);
      //   oriY = 0;
      //   currCase++;
      // }
      //------------------wiggle code----------------------
    }
  else if (currCase == 1){ //find ramp
      driveTime(-currL, -currR);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      oriY += gyroOut.y*(millis()-lastT)/1010;
      lastT = millis();
      myPID.Compute();

      if (oriY >= 10){
        currCase++;
        delay(500);
        oriX = 0;
        lastT = millis();
      }
    }
  
  else if(currCase == 2){ //ascend
    driveTime(-5, -5);
    // gyroOut = readGyro(oriY, oriY, oriZ);
    // oriZ += gyroOut.z*(millis()-lastT)/1010;

    // myPID.Compute();
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY <= -5){ //if ultra finds top
      currCase++;
      driveTime(-200, -200);
      delay(1500);
      lastT = millis();
      oriY = 0;
    }
  }
  else if (currCase == 3){ //top of ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY <= -5){ //if ultra finds ramp down
      driveTime(-220, -220);
      currCase++;
      lastT = millis();
      delay(250);
      oriY = 0;
    }
  }
  else if (currCase == 4){ //descend
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY <= -5){ //if gyro reads 0 on y (yaw? idk lmao)
      // turn(90);
      driveTime(-220, -220);
      delay(250);
      currCase++;
      lastT = millis();
      // currL = 150;
      // currR = 150;
      oriY = 0;
    }
  }

  else if (currCase == 5){ //get off ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY >= 10){ //if gyro finds flat ground
      stop_motors();
      delay(500);
      turn(-90);
      // currL = 150;
      // currR = 150;
      currCase++;
      stop_motors();
    }
  }
}
  // case 5: //go to pole
//     driveTime(currL, currR);
//     gyroOut = readGyro(oriY, oriY, oriZ);
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
  // Axes gyroOut = readGyro(oriY, oriY, oriZ);
  // oriY = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;
  double lastT = millis();
  Axes gyroOut;
  if(deg >= 0){
    driveLeft(-225); //new: 70 old: -60
    driveRight(225); //new: -70 old: 52
    while(oriZ < (deg*0.96)){
      gyroOut = readGyro(oriY, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1000; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(225); //new: 70 old: 60
    driveRight(-225); //new: -70 old: -52
    while(oriZ > (deg*0.96)){
      gyroOut = readGyro(oriY, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1000;
      lastT = millis();
      // delay(10);
    }
  }
  stop_motors();
  oriY = 0;
  oriY = 0;
  oriZ = 0;
}