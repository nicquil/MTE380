#include "motor_defs.h"
#include "Arduino.h"

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

void motor_setup(){
  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
void turn_move(int speed, bool direction){
  analogWrite(enA, speed);
	analogWrite(enB, speed);  

  digitalWrite(in1, direction);
  digitalWrite(in2, !direction);
  digitalWrite(in3, direction);
  digitalWrite(in4, !direction);  
}
void straight_move(int speed, bool direction){
  analogWrite(enA, speed);
	analogWrite(enB, speed);  

  digitalWrite(in1, direction);
  digitalWrite(in2, !direction);
  digitalWrite(in3, direction);
  digitalWrite(in4, !direction);    
}

void stop_motors(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
