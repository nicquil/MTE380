#include "motor_defs.h"
#include "Arduino.h"


//----------------------------------------------------------NEW MOTOR CODE---------------------------------------------------------------

// // Motor A connections - right motor
// int enA = 9; //PWMA
// int inA = 8; //directionA
// // Motor B connections - left motor
// int enB = 3; //PWMB
// int inB = 4; //directionB

// void motor_setup(){
//   // Set all the motor control pins to outputs
// 	pinMode(enA, OUTPUT);
// 	pinMode(enB, OUTPUT);
// 	pinMode(inA, OUTPUT);
// 	pinMode(inB, OUTPUT);
	
// 	// Turn off motors - Initial state
// 	// digitalWrite(inA, LOW);
// 	// digitalWrite(inB, LOW);
//   analogWrite(enA, 255);  
//   analogWrite(enB, 255);  

// }
// void move_forward(){
//   analogWrite(enA, 192);  
//   digitalWrite(inA, 1);  

//   analogWrite(enB, 200);  
//   digitalWrite(inB, 0);  
// }
// void move_backward(){
//   analogWrite(enA, 200);  
//   digitalWrite(inA, 0);  

//   analogWrite(enB, 200);  
//   digitalWrite(inB, 1);  
// }
// void stop_motors(){
//   analogWrite(enA, 255);  
//   analogWrite(enB, 255);    
// }


// void driveLeft(int speed){
//   if (speed >= 0){
//     analogWrite(enB, speed);
//     digitalWrite(inB, 0);
//   }
//   else{
//     analogWrite(enB, -speed);
//     digitalWrite(inB, 1);
//   }
// }
// void driveRight(int speed){
//   if (speed >= 0){
//     analogWrite(enA, speed);
//     digitalWrite(inA, 1);
//   }
//   else{
//     analogWrite(enA, -speed);
//     digitalWrite(inA, 0);
//   }
// }

//-----------------------------------------------------------------------OLD MOTOR CODE------------------------------------------------------------------------------------------------
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
  digitalWrite(in3, !direction);
  digitalWrite(in4, direction);    
}

void stop_motors(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void driveLeft(int speed){
  if (speed >= 0){
    analogWrite(enB, speed);
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
  }
  else{
    analogWrite(enB, -speed);
    digitalWrite(inB, 0);
    digitalWrite(inB, 1); 
  }
}
void driveRight(int speed){
  if (speed >= 0){
    analogWrite(enA, speed);
    digitalWrite(inA, 1);
    digitalWrite(inA, 0);
  }
  else{
    analogWrite(enA, -speed);
    digitalWrite(inA, 0);
    digitalWrite(inA, 1);
  }
}

