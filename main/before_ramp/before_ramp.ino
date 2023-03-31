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

double currL = 210;
double currR = 210;

float TOF_TOLERANCE = 10; // 15cm on tolerance for distance detection
// scenario flags
bool TO_RAMP = 0;
bool ON_RAMP = 0;
bool DOWN_RAMP = 0;
bool DRIVE_FROM_RAMP = 1;

// signal flags
bool DIST_DETECTED = 0;
bool BASE_DETECTED = 0;
bool RAMP_DETECTED = 0;
bool POLE_SEEN = 0;
bool POLE_FOUND = 0;


PID myPID(&oriZ, &currL, &heading, kP, kI, kD, P_ON_M, DIRECT); //new: DIRECT

//---------------------------------------SETUP FUNCTION---------------------------------------

void setup(void) {
  motor_setup();
  setup_serial_i2c();

  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Setup for Sensors
  setupUltra();
  setupGyro();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
}


//---------------------------------------MAIN FUNCTION---------------------------------------

void loop() {
  // detectDistance(40, 3, 30);
  // Serial.println("Distance Detected");    
  // delay(5000);
  // move(180, 180, 1);
  // delay(5000);
  // stop_motors();
  if (TO_RAMP == 1) {
    // initial move for 2sec
    driveTime(currL, currR);

    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    myPID.Compute();

    delay(1000);
    lastT = millis();
    // begin detection for distance
    while (RAMP_DETECTED == 0) {
      if(detectDistance(40, 3, 15)) { 
        // detectDistance(210, 3, 25);    
        //detectDistance(30, 5, 25);
        // delay(250);
        stop_motors();
        Serial.println("STOP MOTORS");
        delay(1000);
        // turn 90 to align
        oriZ = 0;
        turn(-90);  
        delay(1000); 
        RAMP_DETECTED = 1; 
      }
    }  
    driveTime(-currL, -currR);
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    myPID.Compute();
    delay(1000);

    while (TO_RAMP == 1) {
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriY += gyroOut.y*(millis()-lastT)/1010;
      lastT = millis();
      // driveTime(150, 150);
      if(oriY <= 10){ //if gyro finds ramps
        delay(200);
        oriY = 0;
        stop_motors();
      }
    }
    delay(5000);
    // when gyro changes, flag that the robot is on the ramp
    TO_RAMP = 0;
    // ON_RAMP = 1;
  }
  if (DRIVE_FROM_RAMP == 1) {
    int count_readings = 0;
    bool first_reading = 1;
    float prev_read = 0;
    float distance = 0;
    lastT = millis();
    driveTime(currL, currR);
    // check pole far away
    while (POLE_SEEN == 0) {
      // PID movement as it searches
      Axes gyroOut = readGyro(oriX, oriY, oriZ);
      oriZ += gyroOut.z*(millis()-lastT)/1010;
      myPID.Compute();
      driveTime(currL, currR);
      ////-----------------GOOD DETECT POLE FROM RYAN----------------------
      // check on ultrasonic readings
      if (millis()-lastT > 3000) {
        distance = readUltra();
        if (first_reading == 1) {
          prev_read = distance;
          first_reading = 0;
        }
        // checking from back of boundary
        if (distance < 190 && distance > 10){ // max diagonal is 250cm, to wall is 220cm (post is 60x60cm)
        // for straight drive back, detect within 190cm
        // for diagonal scan, detect within 220cm
          // float delay_ms = 0;
          if ((distance > (prev_read - TOF_TOLERANCE)) && (distance < (prev_read + TOF_TOLERANCE))) {
            count_readings++;
            if (count_readings == 3){
              stop_motors();     
              delay(1000);
              oriZ = 0;
              turn(90);
              DRIVE_FROM_RAMP = 0;
              POLE_SEEN = 1;
            }
          }
          else {
            prev_read = distance;
          }
        }
      }
    }
  }
  // just go to pole at the back
  if (POLE_SEEN == 1){
    delay(2000);
    find_pole();
    stop_motors();
    POLE_SEEN = 0;
  }
  // if (ON_RAMP == 1){
  //   // when on ramp, increase speed to 140
  //   driveTime(140, 140); // 5
  //   // reduce speed to 180 after 5 seconds (gyro change flag?)
  //   int speedA = 180;
  //   int speedB = 180;
  //   driveTime(speedA, speedB);
  //   while (speedA < 220) {
  //     speedA = speedA + 20;
  //     speedB = speedB + 20;
  //     move(speedA, speedB, 1); //speed, direction: 1->cw 0->ccw
  //     delay(3000);
  //   }    
  //   stop_motors(); // hope it stops here to slide down
  //   ON_RAMP = 0;
  //   DRIVE_FROM_RAMP = 1;
  // }
//   while(BASE_DETECTED == 0) {
//     if (DRIVE_FROM_RAMP == 1) {
//       driveTime(500, 180, 180); // drive forward a bit
//       turn(90); 
//       lastT = millis();
//       //
//       while ((millis()-lastT) < 4000) {
//         if (!RAMP_DETECTED) {
//           driveTime(4000, 180, 180);
//         }
//         else {
//           delay(250);
//           stop_motors();
//           BASE_DETECTED = 1;
//         }
//       }
//       stop_motors();
//       delay(100);
//       turn(90);
//     }
//   }
// }
}


//---------------------------------------DETECT KNOWN DISTANCE---------------------------------------

bool detectDistance(float distance, int min_detections, int poll_rate){
  int detect_count = 0;
  int bad_detect = 0;
  while (detect_count < min_detections) {
    float measured_dist = readUltra(); // read in cm
    if ((measured_dist < 2) || (measured_dist > 200)){
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
    else {
      detect_count = 0;
    }
    delay(poll_rate);
  }
  return true;
}

//---------------------------------------DRIVING AND TURNING FUNCTIONS---------------------------------------

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
    driveLeft(225); //new: 70 old: 60
    driveRight(-225); //new: -70 old: -52
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

void find_pole(){
  //turn the motors on
  driveTime(150, 150);

  while (POLE_FOUND == false){
    Axes gyroOut = readGyro(oriX, oriY, oriZ);
    oriX += gyroOut.x*(millis()-lastT)/1010;
    oriY += gyroOut.y*(millis()-lastT)/1010;
    oriZ += gyroOut.z*(millis()-lastT)/1010;
    myPID.Compute();
    driveTime(currL, currR);
    lastT = millis();
    Serial.println(oriX);
    if((oriX <= -2) || (abs(oriY) > 3)){
      // driveTime(230, 230);
      // delay(3000);
      stop_motors();
      POLE_FOUND = true;
      oriX = 0;
      oriY = 0;
      //delay(10000);
      //lastT = millis();
    }
  }
}
