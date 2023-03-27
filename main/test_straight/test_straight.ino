// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
// #include <Adafruit_ICM20X.h>
// #include <Adafruit_ICM20948.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
// #include "readGyroscope.h"
#include "sensor_functions.h"
#include "motor_defs.h"
#include "temporary_functions.h"


int TEST_COMPLETED = 0;

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  
  // setupTOF();
  // setupGyro();
}


void loop() {
  int speedA = 180;
  int speedB = 180;
  move(speedA, speedB, 1); //speed, direction: 1->cw 0->ccw  
  while (speedA > 100) {
    speedA = speedA - 20;
    speedB = speedB - 20;
    move(speedA, speedB, 1); //speed, direction: 1->cw 0->ccw
    delay(3000);
  }
  stop_motors(); 
}
