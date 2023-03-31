
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "readGyroscope.h"

Adafruit_ICM20948 icm;
// Axes gyroOut;

float rotationThreshold = M_PI/180;

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
  
  orX = float(gyro.gyro.x)*180/M_PI;
  
  orY = float(gyro.gyro.y)*180/M_PI;

  orZ = float(gyro.gyro.z)*180/M_PI;
  
  Axes a;
  a.x = orX;
  a.y = orY;
  a.z = orZ;

  return a;
}

void setupGyro(){
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  //  icm.setGyroRateDivisor(255);
}

