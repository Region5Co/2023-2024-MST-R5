
#include "Robot.h"
#include "IEEE_Pinout.h"
#include "HCSR04.h"


Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE);
HCSR04 hc(HC_TRIGGER, HC_ECHO)



Robot robot(fl, fr, br, bl);

void setup() {
  robot.init();
}

void loop() {
  testUltra();
  delay(500);
}

void testUltra() {
  Serial.println(hc.dist());
}

void square() {
  robot.drive(FORWARD, 100, 1000);
  robot.drive(RIGHT, 100, 1000);
  robot.drive(BACKWARD, 100, 1000);
  robot.drive(LEFT, 100, 1000);
}

void moveUntil(moveDirection dir, int targetDist) {
  while (hc.dist() > targetDist) {
    robot.drive(dir, 100);
  }
  robot.stop();
}