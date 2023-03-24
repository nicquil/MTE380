// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Arduino.h>
#include <Wire.h>
#include "sensor_functions.h"
#include "motor_defs.h"
#include "temporary_functions.h"

bool CLOSE_DETECTED = 0;
bool MED_DETECTED = 0;
bool FAR_DETECTED = 0;
float MIN_DETECTION_COUNT = 6;
float MIN_DETECTION_COUNT_MED = 4;
float MIN_DETECTION_COUNT_FAR = 3;

// float MIN_DISTANCE_TOL = 25; // in mm (tol to determine a reading is similar / same distance)

void setup(void) {
  motor_setup();
  setup_serial_i2c();
  setupTOF();
}


void loop() {
  int closeCt = 0;
  int medCt = 0;
  int farCt = 0;
  float prev_dist = 0;
  float distance = 0;
  // regular speed
  move(221, 224, 1); //speed a and b , direction: 1->cw 0->ccw
   //speed a and b , direction: 1->cw 0->ccw

  // wait to initialize
  delay(2000);

  while (CLOSE_DETECTED == 0 || MED_DETECTED == 0 || FAR_DETECTED == 0){
    // speed up if reading from 0.1m to 0.6m obstacle
    delay(25);
    distance = readTOF();
    if ((distance < 50) || (distance > 2000)){
      closeCt = 0;
      medCt = 0;
      farCt = 0;
    }
    // else if ((distance > 100) && (distance < 630) && (CLOSE_DETECTED == 0)) {   
    else if ((distance > 475) && (distance < 525) && (CLOSE_DETECTED == 0)) {    
      closeCt++;
      medCt = 0;
      farCt = 0;
      Serial.println("close CT incremented");
      // if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
      //   medCt = 0;
      //   farCt = 0;
      //   prev_dist = distance;
      //   Serial.println("TWO CONSECUTIVE CLOSE RANGES DETECTED");      
      // }
      if(closeCt == MIN_DETECTION_COUNT) {
        move(155, 170, 1); //speed a and b , direction: 1->cw 0->ccw
        CLOSE_DETECTED = 1;
        closeCt = 0;
        Serial.println("SLOW SPEED ACTIVATED (CLOSE RANGE)");
      }
    } 
    else if ((distance > 1175) && (distance < 1225) && (MED_DETECTED == 0)) {   
      medCt++;
      closeCt = 0;
      farCt = 0;
      Serial.println("med CT incremented");
      // if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
      //   closeCt = 0;
      //   farCt = 0;
      //   prev_dist = distance;
      //   Serial.println("TWO CONSECUTIVE MED RANGES DETECTED");      
      // }
      if(medCt == MIN_DETECTION_COUNT_FAR) {
        move(221, 224, 1); //speed a and b , direction: 1->cw 0->ccw
        MED_DETECTED = 1;
        medCt = 0;
        Serial.println("MED SPEED ACTIVATED (MED RANGE)");

      }
    }
    else if ((distance > 1875) && (distance < 1925) && (FAR_DETECTED == 0)) {   
      farCt++;
      closeCt = 0;
      medCt = 0;
      Serial.println("far CT incremented");
      // if (abs(distance - prev_dist) < MIN_DISTANCE_TOL) {
      //   closeCt = 0;
      //   medCt = 0;
      //   prev_dist = distance;
      //   Serial.println("TWO CONSECUTIVE FAR RANGES DETECTED");      
      // }
      if(farCt == MIN_DETECTION_COUNT_FAR) {      
        stop_motors();
        FAR_DETECTED = 1;
        farCt = 0;
        Serial.println("MOTORS STOPPED (FAR RANGE)");
        delay(10000);
      }
    }
  }
  stop_motors();
}
