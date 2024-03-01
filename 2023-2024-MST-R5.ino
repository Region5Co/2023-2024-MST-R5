
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
  /*robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);*/
  switch(stage){
    case 0:
    {
      if(robot.right_distance >= inches)
      {
        robot.drive_right(forward);
      }else{
        robot.stop();
        stage = 1;
      }
      break;
    }
    case 1:
    {
      if(robot.forward_distance() >= y_inches)
      {
        robot.move(forward);
      }else
      {
        robot.stop();
        stage = 2;
      }
    }
  }
}


