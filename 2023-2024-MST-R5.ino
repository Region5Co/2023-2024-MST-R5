#include "Robot.h"

Motor fl(2, 4, 3);
Motor fr(7, 8, 5);
Motor br(10, 11, 6, true);
Motor bl(12, 13, 9, true);

Robot robot(fl, fr, br, bl);

void setup() {
  robot.init();
}

void loop() {
  robot.drive(FORWARD, 100, 3000);
  robot.drive(BACKWARD, 100, 3000);
  robot.drive(LEFT, 100, 3000);
  robot.drive(RIGHT, 100, 3000);
  robot.turn(CW, 100, 1000);
  robot.turn(CCW, 100, 1000);
}

