// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "readGyroscope.h"

// Adafruit_ICM20948 icm;
// uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
float oriX = 0;
float oriY = 0;
float oriZ = 0;
// float rotationThreshold = M_PI/180;
// float rotationThreshold = 0.015;

// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  //  icm.setGyroRateDivisor(255);

  Serial.println();

}



void loop() {
  int usOut = readUltra();
  Axes gyroOut = readGyro(oriX, oriY, oriZ);

  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

  if (usOut != 0){
    
    oriX = gyroOut.x;
    oriY = gyroOut.y;
    oriZ = gyroOut.z;

    Serial.print("Orientation X: ");
    Serial.println(oriX);
    Serial.print("Orientation Y: ");
    Serial.println(oriY);
    Serial.print("Orientation Z: ");
    Serial.println(oriZ);

  }

    Serial.print("\nUltrasonic: ");
    Serial.println(usOut);
    Serial.println();

  delay(10);

}

