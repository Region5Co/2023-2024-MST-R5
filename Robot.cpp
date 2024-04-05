#include "Robot.h"


//@brief
//@param fl,fr,br,bl represents motor in respective position 
//
Robot::Robot(Motor fl, Motor fr, Motor br, Motor bl) {
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
}

//@brief Initializes motors for the Robot
//
void Robot::init() {
  fl.init();
  this->fr.init();
  this->br.init();
  bl.init();
}


//@brief Sets the reverses constant for each motor
//@param fl,fr,br,bl boolean for rather the motor should be reversed 
//
void Robot::reverseMotors(bool fl, bool fr, bool br, bool bl) {
  this->fl.setReversed(fl);
  this->fr.setReversed(fr);
  this->br.setReversed(br);
  this->bl.setReversed(bl);
}

//@brief Stops the Robot motors
//
void Robot::stop() {
  fl.stop();
  fr.stop();
  br.stop();
  bl.stop();
}

//@brief Moves the robot in passed direction with an unanumous speed
//@param direction Requires an input of enum @ref moveDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//
void Robot::drive(moveDirection direction, int speed) {
    switch(direction) {
      case FORWARD:
        clearAllEncCount();
        drive(Kp*speed, Kp*speed, Kp*speed, speed);
        Update_Pos(FORWARD);
        break;

      case BACKWARD:
        clearAllEncCount();
        drive(-Kp*speed, -Kp*speed, -Kp*speed, -speed);
        Update_Pos(BACKWARD);
        break;
        
      case RIGHT:
        clearAllEncCount();
        drive(-Kp*speed, Kp*speed, -Kp*speed, speed);
        Update_Pos(RIGHT);
        break;

      case LEFT:
        clearAllEncCount();
        drive(Kp*speed, -Kp*speed, Kp*speed, -speed);
        Update_Pos(LEFT);
        break;
      default:
        break;
    }
}

//@brief Moves the robot wheels with independent speed values
//@param fl,fr,br,bl Integer representing velocity ranging [-100,100]
//
void Robot::drive(int fl, int fr, int br, int bl) {
  this->fl.run(fl);
  this->fr.run(fr);
  this->br.run(br);
  this->bl.run(bl);
}

//@brief Moves the robot in passed direction with an unanumous speed
//@param direction Requires an input of enum @ref moveDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//@param duration Time in milliseconds to allow robot to drive
//
void Robot::drive(moveDirection direction, int speed, int duration) {
  drive(direction, speed);
  delay(duration);
  stop();
}

//@brief Drives robot with motors at independent speeds for a set time
//@param fl,fr,br,bl Integer representing velocity ranging [-100,100]
//@param duration Time in milliseconds to allow robot to drive
//
void Robot::drive(int fl, int fr, int br, int bl, int duration) {
  drive(fl, fr, br, bl);
  delay(duration);
  stop();
}

//@brief Moves the robot in the passed direction about its center
//@param direction Requires an input of enum @ref turnDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//
void Robot::turn(turnDirection direction, int speed) {
  switch(direction) {
    case CW:
      drive(speed, speed, -speed, -speed);
      break;

    case CCW:
      drive(-speed, -speed, speed, speed);
      break;
    default:
      break;
  }
}

//@brief Moves the robot in the passed direction about its center
//@param direction Requires an input of enum @ref turnDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//@param duration Time in milliseconds to allow robot to drive
//
void Robot::turn(turnDirection direction, int speed, int duration) {
  turn(direction, speed);
  delay(duration);
  stop();
}

float Robot::Get_X_Pos(){
  return fl.getEncoder()->getEncCount();
}

float Robot::Get_Y_Pos(){
  return Y_Pos;
}

void Robot::clearAllEncCount(){
  fl.getEncoder()->clearEncCount();
  fr.getEncoder()->clearEncCount();
  bl.getEncoder()->clearEncCount();
  br.getEncoder()->clearEncCount();
}

void Robot::Update_Pos(moveDirection direction){
  switch(direction){
    case FORWARD:
      Y_Pos += (abs(fl.getEncoder()->getEncDist()) + abs(fr.getEncoder()->getEncDist()) + abs(bl.getEncoder()->getEncDist()) + abs(br.getEncoder()->getEncDist()))/4;
      break;
    case BACKWARD:
      Y_Pos -= (abs(fl.getEncoder()->getEncDist()) + abs(fr.getEncoder()->getEncDist()) + abs(bl.getEncoder()->getEncDist()) + abs(br.getEncoder()->getEncDist()))/4;
      break;
  }
}

