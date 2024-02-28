#include "Robot.h"


Motor bl(D5, D6, D0, false);
Motor fl(D7, D8, D0, false);
Motor fr(D4, D3, D0, false);
Motor br(D1, D2, D0, false);

Robot robot(fl, fr, br, bl);
bool rev =false;
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  robot.init();
  Serial.println("Initialized");
}

void loop() {
  int i=76;
  //rev= !rev;
  robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);
 
}

