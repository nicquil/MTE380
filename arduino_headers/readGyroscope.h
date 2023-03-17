
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;

struct Axes{
  float x;
  float y;
  float z;
};

Axes readGyro(float orX, float orY, float orZ) {
  
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  /* Display the results (acceleration is measured in m/s^2) */

  if (gyro.gyro.x <= rotationThreshold && gyro.gyro.x >= -rotationThreshold){
    gyro.gyro.x = 0;      
  }
  if (gyro.gyro.y <= rotationThreshold && gyro.gyro.y >= -rotationThreshold){
    gyro.gyro.y = 0;      
  }
  if (gyro.gyro.z <= rotationThreshold && gyro.gyro.z >= -rotationThreshold){
    gyro.gyro.z = 0;      
  }
  
  orX += float(gyro.gyro.x)*3.6/M_PI;
  
  orY += float(gyro.gyro.y)*3.6/M_PI;

  orZ += float(gyro.gyro.z)*3.6/M_PI;
  
  Axes a;
  a.x = orX;
  a.y = orY;
  a.z = orZ;

  return a;
}

int readUltra(){
  int reading = 0;
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in "centimeters" (0x51)
  // use 0x51 for centimeters
  // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  // delay(70);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    return reading;
  }

};

