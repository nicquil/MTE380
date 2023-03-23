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
}


void loop() {
  bool closeDetected = 0;
  bool medDetected = 0;
  bool farDetected = 0;
  int closeCt = 0;
  int medCt = 0;
  int farCt = 0;
  // regular speed
  // straight_vary(70, 78, 1); //speed a and b , direction: 1->cw 0->ccw

  // wait to initialize
  delay(2000);

  while (closeDetected == 0 || medDetected == 0 || farDetected == 0){
    // speed up if reading from 0.1m to 0.6m obstacle
    if ((readTOF() > 100) && (readTOF() < 600) && (closeDetected == 0)) {   
      closeCt++;
      if(closeCt == 3) {
        straight_vary(50, 57, 1); //speed a and b , direction: 1->cw 0->ccw
        // delay(500);
        closeDetected = 1;
        closeCt = 0;
      }
    } 
    else if ((readTOF() > 700) && (readTOF() < 1300) && (medDetected == 0)) {   
      medCt++;
      if(medCt == 3) {
        straight_vary(75, 88, 1); //speed a and b , direction: 1->cw 0->ccw
        // delay(500);
        medDetected = 1;
        medCt = 0;
      }
    }
    else if ((readTOF() > 1400) && (readTOF() < 2000) && (farDetected == 0)) {   
      farCt++;
      if(medCt == 3) {      
        stop_motors();
        // delay(5000);
        farDetected = 1;
        farCt = 0;
        delay(10000);
      }
    }
  }
}
