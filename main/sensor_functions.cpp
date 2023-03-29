/*!
 * @file sensor_functions.cpp
 * @brief Compilation of sensor function definitions
 *
 * @author 
 * @version  V1.0
 * @date  2023-03-17
 */

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

DFRobot_VL53L0X sensor;
// Ultrasonic connections
const int TRIG_PIN = 19;
const int ECHO_PIN = 18;
// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

// setup serial communication and join i2c bus
void setup_serial_i2c(){
  // common for wire and serial
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
}

void setupUltra(){
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Set Echo pin as input to measure the duration of 
  //pulses coming back from the distance sensor
  pinMode(ECHO_PIN, INPUT);
}

// setup tof vl53l0x sensor
void setupTOF() {
  //initialize serial communication at 9600 bits per second:
  // Serial.begin(9600);
  //join i2c bus (address optional for master)
  // Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //setMode with eContinuous / eSingle and eLow / eHigh (Single Reading not useful for our application)
  //Case 1: Set to Back-to-back mode and high precision mode (.25/.50/.75 mm accuracy -> unnecessary for our application)
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  //Case 1: Set to Back-to-back mode and low precision mode (nearest mm ex. 8.8cm -> reports 88.00mm)
  //sensor.setMode(sensor.eContinuous,sensor.eLow);

  //Laser rangefinder begins to work
  sensor.start();
 
  Serial.print("Begin Test for Sensor Detection: \n");
  //delay(500);
}

float readUltra(){
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    Serial.println("Out of range");
  } else {
    Serial.print(cm);
    Serial.print(" cm \t");
    Serial.print(inches);
    Serial.println(" in");
  }
  return cm;
}

// read nnnn.nn mm of TOF sensor distance detection
int readTOF() { //frequency in ms for reading collection
  //Get the distance (in mm) every 2s
  Serial.print("TOF Distance: ");Serial.print(sensor.getDistance());
  Serial.println(" mm");
  //The delay is added to demonstrate the effect, and if you do not add the delay,
  //it will not affect the measurement accuracy
  return sensor.getDistance();
}
