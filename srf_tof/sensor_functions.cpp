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

// setup serial communication and join i2c bus
void setup_serial_i2c(){
  // common for wire and serial
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
}

// setup ultrasonic srf02 sensor
void setupUltrasonic(){
  // Wire.begin();                // join i2c bus (address optional for master)
  // Serial.begin(9600);          // start serial communication at 9600bps

  // move to readUltrasonic()
  // step 1: instruct sensor to read echoes
  // Wire.beginTransmission(112); // transmit to device #112 (0x70)
  // // the address specified in the datasheet is 224 (0xE0)
  // // but i2c adressing uses the high 7 bits so it's 112
  // Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  // Wire.write(byte(0x51));      // command sensor to measure in "centimeters" (0x51)
  // // use 0x51 for centimeters
  // // use 0x52 for ping microseconds
  // Wire.endTransmission();      // stop transmitting

  // // step 2: wait for readings to happen
  // delay(65);                   // datasheet suggests at least 65 milliseconds

  // // step 3: instruct sensor to return a particular echo reading
  // Wire.beginTransmission(112); // transmit to device #112
  // Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  // Wire.endTransmission();      // stop transmitting

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

// read nnn cm of ultrasonic sensor distance detection
int readUltrasonic() { //frequency in ms for reading collection
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in "centimeters" (0x51)
  // use 0x51 for centimeters
  // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(65);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  int reading = 0;
  
  // step 4: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print("Ultrasonic Distance: ");Serial.print(reading);   // print the reading
    Serial.println(" cm");
  }
  return reading;
}

// read nnnn.nn mm of TOF sensor distance detection
int readTOF() { //frequency in ms for reading collection
  //Get the distance (in mm) every 2s
  Serial.print("TOF Distance: ");Serial.println(sensor.getDistance());
  Serial.println(" mm");
  //The delay is added to demonstrate the effect, and if you do not add the delay,
  //it will not affect the measurement accuracy
  return sensor.getDistance();
}
