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
  driveTime(3000);
  turn(180);
  driveTime(3000);
  turn(180);  
  // driveLeft(220);
  // driveRight(220);
  // delay(8000);
  // stop_motors();
  // test1();
  // test2(); //90deg test
  // test3(); //180deg test
  // test4(); //360deg test
  delay(5000);
}

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
  myPID.SetSampleTime(10);
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
  delay(3000);
  
  turn(90);
  delay(3000);

  turn(90);
  delay(3000);

  turn(90);
  delay(3000);

}

void test3(){
  turn(180);
  delay(1000);

  turn(-180);
  delay(1000);

}

void test4(){
  turn(360);
  delay(1000);

  turn(-360);
  delay(1000);

}