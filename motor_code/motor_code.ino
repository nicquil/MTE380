#include "motor_defs.h"

void setup() {
  // put your setup code here, to run once:
  motor_setup();
}

void loop() {

  // put your main code here, to run repeatedly:
  //straight_move(100, 1); //speed, direction: 1->forward 0->backwards
  //straight_move();
  //delay(1000);
  //stop_motors();
  //delay(2000);
  
  straight_move(230, 1); //speed, direction: 1->cw 0->ccw
  delay(4000);
  stop_motors();
  delay(2000);
  
  //straight_move(50, 0);
  //delay(1000);
  //stop_motors();
  //delay(2000);
  
}


