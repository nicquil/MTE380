#include "motor_defs.h"


void setup() {
  // put your setup code here, to run once:
  motor_setup();
}

void loop() {
  move_forward();
  delay(8000);
  stop_motors();
  delay(1000);
  // move_backward();
  // delay(2000);
  // stop_motors();
  // delay(1000);

}


