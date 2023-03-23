// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
#include <Wire.h>
#include "sensor_functions.h"
#include "motor_defs.h"
#include "temporary_functions.h"

bool CLOSE_DETECTED = 0;
bool MED_DETECTED = 0;
bool FAR_DETECTED = 0;
int MIN_DETECTION_COUNT = 3;
int MIN_DISTANCE_TOL = 25; // in mm (tol to determine a reading is similar / same distance)

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  setupTOF();
}


void loop() {
  int closeCt = 0;
  int medCt = 0;
  int farCt = 0;
  int prev_dist = 0;
  int distance = 0;
  // regular speed
  // straight_vary(70, 78, 1); //speed a and b , direction: 1->cw 0->ccw

  // wait to initialize
  delay(2000);

  while (CLOSE_DETECTED == 0 || MED_DETECTED == 0 || FAR_DETECTED == 0){
    // speed up if reading from 0.1m to 0.6m obstacle
    distance = readTOF();
    if ((distance > 100) && (distance < 630) && (CLOSE_DETECTED == 0)) {   
      closeCt++;
      if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
        medCt = 0;
        farCt = 0;
        prev_dist = distance;
      }
      if(closeCt == MIN_DETECTION_COUNT) {
        straight_vary(50, 57, 1); //speed a and b , direction: 1->cw 0->ccw
        // delay(500);
        CLOSE_DETECTED = 1;
        closeCt = 0;
      }
    } 
    else if ((distance > 670) && (distance < 1330) && (MED_DETECTED == 0)) {   
      medCt++;
      if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
        closeCt = 0;
        farCt = 0;
        prev_dist = distance;
      }
      if(medCt == MIN_DETECTION_COUNT) {
        straight_vary(75, 88, 1); //speed a and b , direction: 1->cw 0->ccw
        // delay(500);
        MED_DETECTED = 1;
        medCt = 0;
      }
    }
    else if ((distance > 1370) && (distance < 2000) && (FAR_DETECTED == 0)) {   
      farCt++;
      if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
        closeCt = 0;
        medCt = 0;
        prev_dist = distance;
      }
      if(medCt == MIN_DETECTION_COUNT) {      
        stop_motors();
        // delay(5000);
        FAR_DETECTED = 1;
        farCt = 0;
        delay(10000);
      }
    }
  }
}
