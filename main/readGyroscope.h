
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

extern Adafruit_ICM20948 icm;

struct Axes{
  float x;
  float y;
  float z;
};

Axes readGyro(float orX, float orY, float orZ);

void setupGyro();
