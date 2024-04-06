#include "Robot.h"
#include "types.h"


//@brief
//@param fl,fr,br,bl represents motor in respective position 
//
Robot::Robot(Motor* fl, Motor* fr, Motor* br, Motor* bl) {
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
  this->target_angle =0.0;
}

//@brief Initializes motors for the Robot
//
void Robot::init() {
  fl->init();
  fr->init();
  br->init();
  bl->init();
  imu->init();
  imu->calibrate();
  #if IEEE_US
    us->init();
  #endif
 
}

//@brief Sets the reverses constant for each motor
//@param fl,fr,br,bl boolean for rather the motor should be reversed 
//
void Robot::reverseMotors(bool fl, bool fr, bool br, bool bl) {
  this->fl->setReversed(fl);
  this->fr->setReversed(fr);
  this->br->setReversed(br);
  this->bl->setReversed(bl);
}

//@brief Stops the Robot motors
//
void Robot::stop() {
  fl->stop();
  fr->stop();
  br->stop();
  bl->stop();
}

//@brief Moves the robot in passed direction with an unanumous speed
//@param direction Requires an input of enum @ref moveDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//
void Robot::drive(moveDirection direction, int speed) {
    switch(direction) {
      case FORWARD:
        clearAllEncCount();
        drive(Kp*speed, Kp*speed, Kp*speed, Kp*speed);
        Update_Pos(FORWARD);
        break;

      case BACKWARD:
        clearAllEncCount();
        drive(-Kp*speed, -Kp*speed, -Kp*speed, -Kp*speed);
        Update_Pos(BACKWARD);
        break;
        
      case LEFT:
        clearAllEncCount();
        drive(-Kp*speed, Kp*speed, -Kp*speed, Kp*speed);
        Update_Pos(LEFT);
        break;

      case RIGHT:
        clearAllEncCount();
        drive(Kp*speed, -Kp*speed, Kp*speed, -Kp*speed);
        Update_Pos(RIGHT);
        break;
      default:
        break;
    }
}


//@brief Moves the robot in passed direction with an unanumous speed
//@param direction Requires an input of enum @ref moveDirection to go desired direction
//@param speed An integer ranging from [-100, 100] for all motors
//
void Robot::drive(int drive, int strafe, float rotation) {
    fl->run(drive+strafe+rotation);
    fr->run(drive-strafe-rotation);
    bl->run(drive-strafe+rotation);
    br->run(drive+strafe-rotation);
}

void Robot::drive(int _drive, int strafe, float rotate, int duration, float dist){
  clearAllEncCount();
  if(_drive > 0){
    float e=0.0,kp=2.3;
    float desired_angle = getAngle();
    currY = Get_Y_Pos();
    while(abs(Get_Y_Pos()-currY) < dist){
      clearAllEncCount();
      e = (getAngle()-desired_angle)*kp;
      drive(_drive, 0, e);
      delay(duration);
      stop();
      Update_Pos(FORWARD);
    }
  } else if (_drive < 0){
    float e = 0.0, kp= 2.3;
    float desired_angle = getAngle();
    currY = Get_Y_Pos();
    while(abs(Get_Y_Pos()-currY) < dist){
      clearAllEncCount();
      e = (getAngle()-desired_angle)*kp;
      drive(_drive, 0, e);
      delay(duration);
      stop();
      Update_Pos(BACKWARD);
    }
  } else if (strafe > 0){

  } else if (strafe < 0){

  } else {

  }
}



//@brief Moves the robot wheels with independent speed values
//@param fl,fr,br,bl Integer representing velocity ranging [-100,100]
//
void Robot::drive(int fl, int fr, int br, int bl) {
  this->fl->run(fl);
  this->fr->run(fr);
  this->br->run(br);
  this->bl->run(bl);
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
/*
void Robot::drive(int fl, int fr, int br, int bl, int duration) {
  drive(fl, fr, br, bl);
  delay(duration);
  stop();
}*/

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

void Robot::turn(turnDirection direction, float degrees, bool test){
  switch(direction) {
    case CW:
      leftDist = 0;
      while(abs(leftDist) < 11.8){
        clearAllEncCount();
        drive(70, -70, -70, 70);
        delay(50);
        stop();
        leftDist += float((fl->getEncoder()->getCurMoveEncDist()+bl->getEncoder()->getCurMoveEncDist())/2.0);
      }
      break;

    case CCW:
      leftDist = 0;
      while(abs(leftDist) < 11.8){
        clearAllEncCount();
        drive(-70, 70, 70, -70);
        delay(50);
        stop();
        leftDist += float((fl->getEncoder()->getCurMoveEncDist()+bl->getEncoder()->getCurMoveEncDist())/2.0);
      }
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

void Robot::turn(float rotation, float desired_angle){
    rotation = (abs(getAngle()-desired_angle)>=180)? 1:-1;
    rotation= 1-getAngle()/desired_angle;
    Serial.println(rotation);
    float kp = 1.9;
    rotation *= (abs(rotation)<10)?0:kp;
    drive(0,0,rotation);

}

float Robot::Get_Y_Pos(){
  return this->Y_Pos;
}

void Robot::clearAllEncCount(){
  fl->getEncoder()->clearEncCount();
  fr->getEncoder()->clearEncCount();
  bl->getEncoder()->clearEncCount();
  br->getEncoder()->clearEncCount();
}

void Robot::Update_Pos(moveDirection direction){
  switch(direction){
    case FORWARD:
      Y_Pos += (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4.0;
      break;
    case BACKWARD:
      Y_Pos -= (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4.0;
      break;
  }
  

}

//@brief Returns an attached Motor
//@param wheel uses WHEEL enum to choose a motor
//
Motor* Robot::getMotor(WHEEL wheel){
  switch(wheel){
    default:
    case WHEEL::FRONT_LEFT:  return this->fl; break;
    case WHEEL::FRONT_RIGHT: return this->fr; break;
    case WHEEL::BACK_LEFT:   return this->br; break;
    case WHEEL::BACK_RIGHT:  return this->br; break;
  };
}
void Robot::addIMU(Gyro* _imu){
  this->imu = _imu;
}

float Robot::getAngle(){
  return this->imu->getGyroZ();
}
