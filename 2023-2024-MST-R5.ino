
#include "Robot.h"
#include "IEEE_Pinout.h"
#include <Servo.h>
//#include <RangeSensor.h>
#include "Adafruit_VL53L1X.h"
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>

Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);
Servo myservo;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(21, 20);

Robot robot(fl, fr, br, bl);

void setup() {
  //Serial.begin(9600);
  //Serial.println("Initializing");
  robot.init();
  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init"));
    Serial.println(vl53.vl_status);
  }
  if (! vl53.startRanging()) {
    Serial.print(F("sucks at ranging"));
    Serial.println(vl53.vl_status);
  }
  vl53.setTimingBudget(500);
  vl53.VL53L1X_SetDistanceMode(2);
  vl53.VL53L1X_SetROI(4, 16);

  myservo.attach(SERVO_PIN);
  Serial.begin(9600);
}

int stage = 0;
  int a = 50; //distance from the right wall
  int b = 50; //distance from wall were facing
  int c = 50; //distance to hit the button?
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
      Serial.println("In stage 0");
      delay(2000);
      moveUntilWithServo(RIGHT, a, true);
      Serial.println("Change to stage 1");
      delay(1000);
      stage = 1;
      break;
    }
    case 1:
    {
      Serial.println("In stage 1");
      delay(2000);
      moveUntilWithServo(FORWARD, b, true);
      stage = 2;
      break;
    }
    case 2:
    {
      Serial.println("In stage 2");
      delay(2000);
      moveUntilWithServo(LEFT, c, false);
      stage = 3;
      break;
    }
    case 3: 
    {
      robot.drive(FORWARD, 100, 1000); /* will need to test this value. Likely should use the encoders to verify*/
      stage = 4;
      break;
    }
    case 4: 
    {
      Serial.println("In stage 4");
      delay(2000);
      robot.drive(BACKWARD,100,500);
      robot.turn(CW,100,1500);
      stage = 0;
      break;
      /* here would just need to back up a small bit*/
       
    }
  }
}


void square() {
  Serial.println("Forward");
  robot.drive(FORWARD, 100, 1000);
  Serial.println("Right");
  robot.drive(RIGHT, 100, 1000);
  Serial.println("Backward");
  robot.drive(BACKWARD, 100, 1000);
  Serial.println("Left");
  robot.drive(LEFT, 100, 1000);
}

void servoTest() {
  myservo.write(20);
  delay(1500);
  myservo.write(150);
  delay(750);
}

void moveUntilLt(moveDirection dir, int targetDist) {
  while (vl53.distance() == -1){
    Serial.println(vl53.distance());
    Serial.print("stuck in -1");
    robot.drive(dir, 100);
    delay(50);
  }
  while (vl53.distance() > targetDist) {
    Serial.println(vl53.distance());
    Serial.print("moveUntilLt");
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {
  while (vl53.distance() < targetDist) {
    Serial.println(vl53.distance());
    Serial.print("moveUntilLt");
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilWithServo(moveDirection dir, int targetDist, bool isLt) { // 90 degree rotation on this lad lmao
  if (dir == FORWARD) {
    myservo.write(20);
  } else {
    myservo.write(150);
  }
  delay(2000);
  
  if (isLt) {
    moveUntilLt(dir, targetDist);
  } else {
  moveUntilGt(dir, targetDist);
  }
}