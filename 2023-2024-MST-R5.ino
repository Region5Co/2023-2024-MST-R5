
#include "Robot.h"
#include "IEEE_Pinout.h"

Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE);




Robot robot(fl, fr, br, bl);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  robot.init();
  Serial.println("Initialized");
}

void loop() {
  robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);
 
}

#include "mecanum_driver.h"

#define R5_ROUND 1
#if R5_ROUND == 2
#define STATEMACHINE 2
#else
#define STATEMACHINE 1
#endif

int pinList[12] = {5,0,6,7,0,8,7,8,9,10,11,12};
//robot::motor test = robot::motor(1, 2, 3);

void setup(){
  robot::robot_4_wheels drive = robot::robot_4_wheels(pinList);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      //drive.move_right(255);
      break;
  }
}

void loop(){
  drive.move_right(100);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:

      break;
  }
}

