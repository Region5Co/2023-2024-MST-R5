#include "Robot.h"


//@brief
//@param fl,fr,br,bl represents motor in respective position 
//
Robot::Robot(Motor* fl, Motor* fr, Motor* br, Motor* bl) {
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
}


//@brief Initializes motors for the Robot
//
void Robot::init() {
  fl->init();
  fr->init();
  br->init();
  bl->init();
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
        drive(speed, speed, speed, speed);
        break;

      case BACKWARD:
        drive(-speed, -speed, -speed, -speed);
        break;
        
      case RIGHT:
        drive(-speed, speed, -speed, speed);
        break;

      case LEFT:
        drive(speed, -speed, speed, -speed);
        break;
      default:
        break;
    }
}

void Robot::drive_Enc(moveDirection direction, int speed, float distance){
  clearAllEncCount();
  switch (direction){
    case FORWARD:
      this->currY = Get_Y_Pos();
      this->leftKp = 1.0;
      this->rightKp = 1.0;
      while(Get_Y_Pos()-currY < distance){
        clearAllEncCount();
        drive(leftKp*speed, leftKp*speed, rightKp*speed, rightKp*speed, 50);
        this->leftDist = float((this->fl->getEncoder()->getCurMoveEncDist()+this->bl->getEncoder()->getCurMoveEncDist())/2.0);
        this->rightDist = float((this->fr->getEncoder()->getCurMoveEncDist()+this->br->getEncoder()->getCurMoveEncDist())/2.0);
        this->distDiff += float(abs(this->leftDist) - abs(this->rightDist));
        if (distDiff < 0 && abs(distDiff) > 0.25){
          if(leftKp > 1.25){
            rightKp -= Kp;
          } else{
            this->leftKp += Kp;
          }
        } else if (distDiff > 0 && abs(distDiff) > 0.25){
          if(rightKp > 1.25){
            leftKp -= Kp;
          } else {
            this->rightKp += Kp;
          }
        } else {

        }
        /*
        Serial.print("Error ");
        Serial.print(Get_Y_Pos()-currY);
        Serial.print(" Distance Diff ");
        Serial.print(this->distDiff);
        Serial.print(" Left Dist ");
        Serial.print(this->leftDist);
        Serial.print(" Right Dist ");
        Serial.print(this->rightDist);
        Serial.print(" Kps ");
        Serial.print(this->leftKp);
        Serial.print(" ");
        Serial.print(this->rightKp);
        Serial.println(" ");*/
        Update_Pos(FORWARD);
      }
    case BACKWARD:
      this->currY = Get_Y_Pos();
      this->leftKp = 1.0;
      this->rightKp = 1.0;
      while(currY - Get_Y_Pos() < distance){
        clearAllEncCount();
        drive(-leftKp*speed, -leftKp*speed, -rightKp*speed, -rightKp*speed, 50);
        this->leftDist = float((this->fl->getEncoder()->getCurMoveEncDist()+this->bl->getEncoder()->getCurMoveEncDist())/2.0);
        this->rightDist = float((this->fr->getEncoder()->getCurMoveEncDist()+this->br->getEncoder()->getCurMoveEncDist())/2.0);
        this->distDiff += float(abs(this->leftDist) - abs(this->rightDist));
        if (distDiff < 0 && abs(distDiff) > 0.25){
          if(leftKp > 1.25){
            rightKp -= Kp;
          } else{
            this->leftKp += Kp;
          }
        } else if (distDiff > 0 && abs(distDiff) > 0.25){
          if(rightKp > 1.25){
            leftKp -= Kp;
          } else {
            this->rightKp += Kp;
          }
        } else {

        }
        /*
        Serial.print("Error ");
        Serial.print(Get_Y_Pos()-currY);
        Serial.print(" Distance Diff ");
        Serial.print(this->distDiff);
        Serial.print(" Left Dist ");
        Serial.print(this->leftDist);
        Serial.print(" Right Dist ");
        Serial.print(this->rightDist);
        Serial.print(" Kps ");
        Serial.print(this->leftKp);
        Serial.print(" ");
        Serial.print(this->rightKp);
        Serial.println(" ");*/
        Update_Pos(BACKWARD);
      }
    case LEFT:
      this->currX = Get_X_Pos();
      this->frontKp = 1.0;
      this->backKp = 1.0;
      while(currX - Get_X_Pos() < distance){
        clearAllEncCount();
        drive(frontKp*speed, -frontKp*speed, backKp*speed, -backKp*speed, 50);
        this->frontDist = float((this->fl->getEncoder()->getCurMoveEncDist()+this->fr->getEncoder()->getCurMoveEncDist())/2.0);
        this->backDist = float((this->bl->getEncoder()->getCurMoveEncDist()+this->br->getEncoder()->getCurMoveEncDist())/2.0);
        this->distDiff += float(abs(this->frontDist) - abs(this->rightDist));
        if (distDiff < 0 && abs(distDiff) > 0.25){
          if(frontKp > 1.25){
            backKp -= Kp;
          } else{
            this->frontKp += Kp;
          }
        } else if (distDiff > 0 && abs(distDiff) > 0.25){
          if(backKp > 1.25){
            frontKp -= Kp;
          } else {
            this->backKp += Kp;
          }
        } else {

        }
        /*
        Serial.print("Error ");
        Serial.print(Get_Y_Pos()-currY);
        Serial.print(" Distance Diff ");
        Serial.print(this->distDiff);
        Serial.print(" Left Dist ");
        Serial.print(this->leftDist);
        Serial.print(" Right Dist ");
        Serial.print(this->rightDist);
        Serial.print(" Kps ");
        Serial.print(this->leftKp);
        Serial.print(" ");
        Serial.print(this->rightKp);
        Serial.println(" ");*/
        Update_Pos(LEFT);
      }
    case RIGHT:
      this->currX = Get_X_Pos();
      this->frontKp = 1.0;
      this->backKp = 1.0;
      while(Get_X_Pos() - currX < distance){
        clearAllEncCount();
        drive(-frontKp*speed, frontKp*speed, -backKp*speed, backKp*speed, 50);
        this->frontDist = float((this->fl->getEncoder()->getCurMoveEncDist()+this->fr->getEncoder()->getCurMoveEncDist())/2.0);
        this->backDist = float((this->bl->getEncoder()->getCurMoveEncDist()+this->br->getEncoder()->getCurMoveEncDist())/2.0);
        this->distDiff += float(abs(this->frontDist) - abs(this->rightDist));
        if (distDiff < 0 && abs(distDiff) > 0.25){
          if(frontKp > 1.25){
            backKp -= Kp;
          } else{
            this->frontKp += Kp;
          }
        } else if (distDiff > 0 && abs(distDiff) > 0.25){
          if(backKp > 1.25){
            frontKp -= Kp;
          } else {
            this->backKp += Kp;
          }
        } else {

        }
        /*
        Serial.print("Error ");
        Serial.print(Get_Y_Pos()-currY);
        Serial.print(" Distance Diff ");
        Serial.print(this->distDiff);
        Serial.print(" Left Dist ");
        Serial.print(this->leftDist);
        Serial.print(" Right Dist ");
        Serial.print(this->rightDist);
        Serial.print(" Kps ");
        Serial.print(this->leftKp);
        Serial.print(" ");
        Serial.print(this->rightKp);
        Serial.println(" ");*/
        Update_Pos(RIGHT);
      }
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
      clearAllEncCount();
      leftDist = 0;
      rightDist = 0;
      while(abs(leftDist) < 11.75){
        drive(speed, speed, -speed, -speed, 50);
        this->leftDist = float((this->fl->getEncoder()->getCurMoveEncDist()+this->bl->getEncoder()->getCurMoveEncDist())/2.0);
        this->rightDist = float((this->fr->getEncoder()->getCurMoveEncDist()+this->br->getEncoder()->getCurMoveEncDist())/2.0);
      }
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
  return this->fl->getEncoder()->getEncCount();
}

float Robot::Get_Y_Pos(){
  return this->Y_Pos;
}

void Robot::clearAllEncCount(){
  this->fl->getEncoder()->clearEncCount();
  this->fr->getEncoder()->clearEncCount();
  this->bl->getEncoder()->clearEncCount();
  this->br->getEncoder()->clearEncCount();
}

void Robot::Update_Pos(moveDirection direction){
  switch(direction){
    case FORWARD:
      Y_Pos += (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4;
      break;
    case BACKWARD:
      Y_Pos -= (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4;
      break;
    case LEFT:
      X_Pos -= (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4;
    case RIGHT:
      X_Pos += (abs(fl->getEncoder()->getCurMoveEncDist()) + abs(fr->getEncoder()->getCurMoveEncDist()) + abs(bl->getEncoder()->getCurMoveEncDist()) + abs(br->getEncoder()->getCurMoveEncDist()))/4;
  }
}

