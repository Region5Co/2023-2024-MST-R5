
#include "Robot.h"
#include "IEEE_Pinout.h"
#include "HCSR04.h"
//#include "Servo.h"

//Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fr(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);
//HCSR04 hc(HC_TRIGGER, HC_ECHO);
HCSR04 hc(D4, D3);
//Servo myservo;

Robot robot(fl, fr, br, bl);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  robot.init();
  //myservo.attach(pin);
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
  robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);
 
}

void moveUntilWithServo(moveDirection dir, int targetDist) {
  //myservo.write(degree);
  delay(15);
  moveUntil(dir, targetDist);
}