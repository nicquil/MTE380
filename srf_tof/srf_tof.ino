/*
Sample code for test the SRF02 with the I2C mode based on Arduino UNO!
Command for reference:http://robot-electronics.co.uk/htm/srf02techI2C.htm
Connection:
SRF02       Arduino
5v Vcc    -> 5V
SDA       -> A4
SCL       -> A5
Mode      -> no connection
0v Ground -> GND
*/

#include <Wire.h>
#include "sensor_functions.h"

void setup()
{
  setup_serial_i2c();
  setupUltrasonic();
  setupTOF();
}


void loop()
{
  readTOF();
  readUltrasonic();
  delay(250);                  // wait a bit since people have to read the output :)
}

