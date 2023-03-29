#include "motor_defs.h"
#include "Arduino.h"

// Motor A connections - right motor
int enA = 9; //PWMA
int inA = 8; //directionA
// Motor B connections - left motor
int enB = 3; //PWMB
int inB = 4; //directionB

// Ultrasonic connections
const int TRIG_PIN = 19;
const int ECHO_PIN = 18;
// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

void motor_setup(){
  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(inA, OUTPUT);
	pinMode(inB, OUTPUT);
	
	// Turn off motors - Initial state
	// digitalWrite(inA, LOW);
	// digitalWrite(inB, LOW);
  analogWrite(enA, 255);  
  analogWrite(enB, 255);  

}
void move(int speedA, int speedB, bool direction){ // right motor speed, left motor speed, forward = 1 / backward = 0
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);

  if(direction == 1) {
    digitalWrite(inA, 1);  
    digitalWrite(inB, 0);  
  }
  else if (direction == 0){
    digitalWrite(inA, 0);  
    digitalWrite(inB, 1);     
  }
}

void stop_motors(){
  analogWrite(enA, 255);  
  analogWrite(enB, 255);    
}

void driveLeft(int speed){
  if (speed >= 0){
    analogWrite(enB, speed);
    digitalWrite(inB, 0);
  }
  else{
    analogWrite(enB, -speed);
    digitalWrite(inB, 1);
  }
}
void driveRight(int speed){
  if (speed >= 0){
    analogWrite(enA, speed);
    digitalWrite(inA, 1);
  }
  else{
    analogWrite(enA, -speed);
    digitalWrite(inA, 0);
  }
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
