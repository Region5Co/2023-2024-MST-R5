
#include "Robot.h"
#include "IEEE_Pinout.h"
#include "HCSR04.h"
#include <Servo.h>

Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);
HCSR04 hc(HC_TRIGGER, HC_ECHO);
Servo myservo;

Robot robot(fl, fr, br, bl);

void setup() {
  //Serial.begin(9600);
  //Serial.println("Initializing");
  robot.init();

  myservo.attach(SERVO_PIN);
}

int stage = 0;
  int a,b,c; //desired distance from the wall
  a = 50; //distance from the right wall
  b = 50; //distance from wall were facing
  c = 50; //distance to hit the button?
void loop() {
  /*robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);*/
  switch(stage){ //make stage variable
    case 0: //need to get data from ultrasonic sensor (this is psuedocode)
    {
      moveUntilWithServo(RIGHT, a, true);
      stage = 1;
      break;
    }
    case 1:
    {
      moveUntilWithServo(FORWARD, b, true);
      stage = 2;
      break;
    }
    case 2:
    {
      moveUntilWithServo(LEFT, c, false);
      robot.drive(FORWARD, 100, 1000); /* will need to test this value. Likely should use the encoders to verify*/
      stage = 3;
      break;
    }
    case 4: 
    {
      robot.drive(BACKWARD,100,500);
      robot.drive(CW,100,1500);
      stage = 0;
      /* here would just need to back up a small bit*/
        
    
  }
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

void servoTest() {
  myservo.write(0);
  delay(1500);
  myservo.write(180);
  delay(750);
}

void moveUntilLt(moveDirection dir, int targetDist) {
  while (hc.dist() > targetDist) {
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {
  while (hc.dist() < targetDist) {
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilWithServo(moveDirection dir, int targetDist, bool isLt) { // 90 degree rotation on this lad lmao
  if (dir == FORWARD) {
    myservo.write(180);
  } else if (dir == RIGHT) {
    myservo.write(0);
  }
  delay(15);
  if (isLt) {
    moveUntilLt(dir, targetDist);
  } else {
    moveUntilGt(dir, targetDist);
  }
}
