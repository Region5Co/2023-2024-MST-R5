#include "Encoder.h"
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

Encoder encFL(E_FRONT_LEFT_INT, E_FORNT_LEFT_DIR);
Encoder encFR(E_FRONT_RIGHT_INT, E_FRONT_RIGHT_DIR);
Encoder encBL(E_BACK_LEFT_INT, E_BACK_LEFT_DIR);
Encoder encBR(E_BACK_RIGHT_INT, E_BACK_RIGHT_DIR);

void interruptEncoderFL(){
  encFL.incEncCount();
}
void interruptEncoderFR(){
  encFR.incEncCount();
}
void interruptEncoderBL(){
  encBL.incEncCount();
}
void interruptEncoderBR(){
  encBR.incEncCount();
}

Robot robot(&fl, &fr, &br, &bl);

void setup() {
  fl.attachEncoder(&encFL);
  fr.attachEncoder(&encFR);
  bl.attachEncoder(&encBL);
  br.attachEncoder(&encBR);
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

  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

  myservo.attach(SERVO_PIN);
  Serial.begin(9600);
}

int stage = 0;
  int a = 170; //distance from the right wall
  int b = 120; //distance from wall were facing
  int c = 305; //distance to hit the button?
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
      /*robot.drive_Enc(FORWARD, 60, 24);
      delay(2000);
      robot.drive_Enc(LEFT, 60, 24);
      delay(2000);
      robot.drive_Enc(BACKWARD, 60, 24);
      delay(2000);
      robot.drive_Enc(RIGHT, 60, 24);
      delay(2000);*/
      robot.turn(CW, 70);
      delay(2000);
      /*
      Serial.println("In stage 0");
      delay(2000);
      moveUntilWithServo(RIGHT, a, true);
      delay(1000);
      */

      /*
      robot.drive(FORWARD, 70, 100);
      
      Serial.print("Encoder: ");
      Serial.print(encFL.getEncDist());
      Serial.print(" ");
      Serial.print(encFR.getEncDist());
      Serial.print(" ");
      Serial.print(encBL.getEncDist());
      Serial.print(" ");
      Serial.print(encBR.getEncDist());
      Serial.println(" ");
      */

      //stage = 1;
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
      delay(1000);

      stage = 3;
      break;
    }
    case 3: 
    {
      Serial.println("In stage 3");
      delay(1000);
      robot.drive(FORWARD, 100, 1500);
      delay(1000);

      stage = 4;
      break;
    }
    case 4: 
    {
      Serial.println("In stage 4");
      delay(1000);
      robot.drive(BACKWARD,70,1000);
      delay(500);
      robot.turn(CW,70,1410);
      delay(500);

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
  /*
  while (vl53.distance() == -1){
    Serial.println(vl53.distance());
    Serial.print("stuck in -1");
    robot.drive(dir, 100);
    delay(50);
  }*/

  while (revisedDist(vl53.distance()) > targetDist) {
    robot.drive(dir, 70);
    delay(10);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {
  while (revisedDist(vl53.distance()) < targetDist) {
    robot.drive(dir, 70);
    delay(10);
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

int revisedDist(int dist){  
    if (dist==-1){
        dist=1000;
    }
    return dist;
}
