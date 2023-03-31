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

// CHANGE THESE VALUES DEPENDING ON POLE POSITION
float MAX_DISTANCE = 120; // 150: STEEL WALL MAX CLOSEST
int poleClose = 3; //1: beside ramp, 2: at back boundary, 3: anywhere else

int currCase = 0;
int currCount = 0;

int count_readings = 0;
float prev_read = 0;
float distance = 0;

double oriX = 0;
double oriY = 0;
double oriZ = 0;

double currL = 210;
double currR = 205;
double heading = 0;

bool first = true;

unsigned long lastT = millis();

PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT); //new: DIRECT


// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

void setup(void) {
  setup_serial_i2c();
  
  while (!Serial){
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  }
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
  // oriX += gyroOut.x*(millis()-lastT)/1010;
  // oriY += gyroOut.y*(millis()-lastT)/1010;
  
  // lastT = millis();
  // Serial.println(oriY);
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
  // oriY += gyroOut.y*(millis()-lastT)/1010;
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
      // driveTime(200, 195);
      // Axes gyroOut = readGyro(oriX, oriY, oriZ);
      // oriZ += gyroOut.z*(millis()-lastT)/1010;
      // // myPID.Compute();
      // lastT = millis();

      // float measured_dist = readUltra();
    
      // if (measured_dist <= 88){
      //   currCount += 1;
      // }
      // else{
      //   currCount = 0;
      // }
      // if (currCount == 3){

        
      //   delay(10);
      //   turn(-90);

      //   delay(1000);
      //   currCase++;   
      //   oriY = 0;
      //   oriX = 0;
      //   oriZ = 0;
      //   lastT = millis();

      //
      
      //}

      //ram in wall
      driveTime(200, 192);
      delay(3200);
      stop_motors();
      delay(500);
      driveTime(-200,-195);
      delay(650);
      turn(-86);
      delay(500);
      oriY = 0;
      oriX = 0;
      oriZ = 0;
      currCase++;
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
      //driveTime(-180, -175);
      driveTime(-200,-195);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      oriY += gyroOut.y*(millis()-lastT)/1010;
      lastT = millis();
      // myPID.Compute();

      if (oriY >= 10){

        currCase++;
        delay(500);
        oriY = 0;
        oriX = 0;
        oriZ = 0;
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
      delay(300);
      lastT = millis();
      oriY = 0;
      oriX = 0;
      oriZ = 0;
    }
  }
  else if (currCase == 3){ //top of ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY <= -5){ //if ultra finds ramp down
      driveTime(-200, -200);
      currCase++;
      currCase++;
      delay(3688); //current: 3688 at 11.5V before: 2230 timed value (idk what V)
      stop_motors();
      delay(800);
      // driveTime(220, 215);
      // delay(200);
      // stop_motors();
      turn(-90);
      currCase++;
      lastT = millis();
      // delay(250);
      oriY = 0;
      oriX = 0;
      oriZ = 0;
    }
  }
  else if (currCase == 4){ //descend
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY <= -5){ //if gyro reads 0 on y (yaw? idk lmao)
      stop_motors();
      delay(500);
      
      // turn(90);
      driveTime(-220, -220);
      delay(500);
      currCase++;
      lastT = millis();
      // currL = 150;
      // currR = 150;
      oriY = 0;
      oriX = 0;
      oriZ = 0;
    }
  }

  else if (currCase == 5){ //get off ramp
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriY += gyroOut.y*(millis()-lastT)/1010;
    
    lastT = millis();
    if(oriY >= 5){ //if gyro finds flat ground
      
      // stop_motors();

      delay(1800);
      stop_motors();
      driveTime(220, 215);
      delay(500);
      stop_motors();
      turn(-90);
      // currL = 150;
      // currR = 150;
      currCase++;
      oriY = 0;
      oriX = 0;
      oriZ = 0;      
      stop_motors();
      lastT = millis();
    }
  }
    else if ((currCase == 6) && (poleClose == 3)){ //find pole
      driveTime(200, 195);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      // myPID.Compute();
      lastT = millis();
      distance = readUltra();
      if (first){
        unsigned long firstT = millis();
        while((millis() - firstT) < 1800){
          driveTime(200, 195);
          // Axes gyroOut = readGyro(oriX, oriY, oriZ);
          // oriZ += gyroOut.z*(millis()-lastT)/1010;
          // myPID.Compute();
          // lastT = millis();
        }
        prev_read = distance;
        first = false;
      }
      else{
        float delay_ms = 0;
        if ((distance < MAX_DISTANCE) && (distance > 10)){
          if((distance > (prev_read - 20)) && (distance < (prev_read + 20))){
            count_readings++;
            if(count_readings >= 3){
              delay_ms = distance*tan(15*PI/180)*250/10;
              delay(delay_ms);
              stop_motors();

              delay(1000);
              oriY = 0;
              oriX = 0;
              oriZ = 0;
              turn(90);
              currCase++;
              lastT = millis();
            }
          }
          else{
            prev_read = distance;            
          }
        }
      }

      

    // if(oriY >= 10){ //if ultra finds pole
    //   stop_motors();
    //   delay(500);
    //   turn(-90);
    //   // currL = 150;
    //   // currR = 150;
    //   currCase++;
    //   stop_motors();
    // }
  }
  else if ((currCase == 6) && (poleClose == 1)){
      driveTime(180, 175);
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      myPID.Compute();
      lastT = millis();
      distance = readUltra();
      unsigned long firstT = millis();

      while(millis() - firstT < 1800){
        driveTime(currL, currR);
        gyroOut = readGyro(oriX, oriY, oriZ);
        oriZ += gyroOut.z*(millis()-lastT)/1010;
        myPID.Compute();
        lastT = millis();
      }

      stop_motors();

      oriZ = 0;
      oriX = 0;
      oriY = 0;
      turn(90);
      currCase++;
      lastT = millis();      
  }
  else if ((currCase == 6) && (poleClose == 2)){
    currCase++;     
  }
  else if (currCase == 7){
    driveTime(130, 125); // earlier working, barely gets on ramp: 180, 175
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    oriY += gyroOut.y*(millis()-lastT)/1010;
    oriX += gyroOut.x*(millis()-lastT)/1010;
    
    lastT = millis();

    if((oriY <= -3) || (abs(oriX) > 3)){      
      stop_motors();
      currCase++;
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
    while(oriZ < (deg)){
      gyroOut = readGyro(oriY, oriY, oriZ);
      
      oriZ += gyroOut.z*(millis()-lastT)/1000; //970: 1000 minus 3 percent tolerance, 1.015: accounting for 1.5 percent tolerance due to cold day, change to 1 when near 25 deg C
      lastT = millis();
      // delay(10);
    }
  }
  else{
    driveLeft(225); //new: 70 old: 60
    driveRight(-225); //new: -70 old: -52
    while(oriZ > (deg)){
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