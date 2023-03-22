// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "readGyroscope.h"
#include "sensor_functions.h"
#include "motor_defs.h"
#include "temporary_functions.h"

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  
  setupTOF();
  setupGyro();
}


void loop() {
  int countDetect = 0;
  straight_vary(70, 78, 1); //speed, direction: 1->cw 0->ccw
  if (50 < readTOF() < 200) {   // have an obstacle within 20cm of TOF sensor to detect the end of the 400cm stretch
  //   countDetect++;
  // }else if (readTOF() > 250) {
  //   countDetect = 0;
  // }

  // if (countDetect >= 2) {
    stop_motors();
    delay(5000);
    // countDetect = 0;
  }
}
