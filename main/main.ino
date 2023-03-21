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
bool TO_RAMP = 0;
bool TURN_TO_RAMP = 0;
bool UP_RAMP = 0;
bool DOWN_RAMP = 0;
bool TURN_FROM_RAMP = 0;
bool BACK_FROM_RAMP = 0;

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
  
  straight_move(70, 1); //speed, direction: 1->cw 0->ccw
  // Axes gyroOut = readGyro(oriX, oriY, oriZ);
  // oriX = gyroOut.x;
  // oriY = gyroOut.y;
  // oriZ = gyroOut.z;

  if (readTOF() <= 200) {
    stop_motors();
    delay(2000);
  }
  //readUltrasonic(); // may be skipping some ultrasonic sensor reads due to byte reading in function (defined in sensor_functions.cpp)

  // delay(10);
}
