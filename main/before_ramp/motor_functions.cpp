#include "motor_defs.h"
#include "Arduino.h"

// Motor A connections - right motor
int enA = 9; //PWMA
int inA = 8; //directionA
// Motor B connections - left motor
int enB = 3; //PWMB
int inB = 4; //directionB

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
void move_forward(int speedA, int speedB){
  analogWrite(enA, speedA);  
  digitalWrite(inA, 1);  

  analogWrite(enB, speedB);
  digitalWrite(inB, 0);  
}
void move_backward(int speedA, int speedB){
  analogWrite(enA, speedA);  
  digitalWrite(inA, 0);  

  analogWrite(enB, speedB);  
  digitalWrite(inB, 1);  
}
void stop_motors(){
  analogWrite(enA, 255);  
  analogWrite(enB, 255);    
}

void move(int speedA, int speedB, bool direction){ // right motor speed, left motor speed, forward = 1 / backward = 0
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);

  if(direction == 1) {
    digitalWrite(inA, 0);  
    digitalWrite(inB, 1);  
  }
  else if (direction == 0){
    digitalWrite(inA, 1);  
    digitalWrite(inB, 0);     
  }
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

