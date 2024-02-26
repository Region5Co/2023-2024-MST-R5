#include "Robot.h"

Robot::Robot(int m1p1, int m1p2, int m1EN, int m2p1, int m2p2, int m2EN, int m3p1, int m3p2, int m3EN, int m4p1, int m4p2, int m4EN) {
  Robot(Motor(m1p1, m1p2, m1EN), Motor(m1p1, m1p2, m1EN), Motor(m1p1, m1p2, m1EN), Motor(m1p1, m1p2, m1EN)); 
}

Robot::Robot(Motor fl, Motor fr, Motor br, Motor bl) {
  type = MECANUM;
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
}

Robot::Robot(int m1p1, int m1p2, int m1EN, int m2p1, int m2p2, int m2EN){
  Robot(Motor(m1p1, m1p2, m1EN), Motor(m2p1, m2p2, m2EN));
}

Robot::Robot(Motor fl, Motor fr) {
  type = TWO_WHEEL;
  this->fl = fl;
  this->fr = fr;
}

void Robot::init() {
  fl.init();
  fr.init();
  
  if(type == MECANUM) {
    br.init();
    bl.init();
  }
}

void Robot::reverseMotors(bool fl, bool fr) {
  this->fl.setReversed(fl);
  this->fr.setReversed(fr);
}

void Robot::reverseMotors(bool fl, bool fr, bool br, bool bl) {
  this->fl.setReversed(fl);
  this->fr.setReversed(fr);
  this->br.setReversed(br);
  this->bl.setReversed(bl);
}

void Robot::stop() {
  fl.stop();
  fr.stop();

  if(type == MECANUM) {
    br.stop();
    bl.stop();
  }
}

void Robot::drive(moveDirection direction, int speed) {
  if(type == MECANUM) {
    switch(direction) {
      case FORWARD:
        drive(speed, speed, speed, speed);
        break;

      case BACKWARD:
        drive(-speed, -speed, -speed, -speed);
        break;
        
      case LEFT:
        drive(-speed, speed, -speed, speed);
        break;

      case RIGHT:
        drive(speed, -speed, speed, -speed);
        break;
    }

    return;
  }

  if(type == TWO_WHEEL) {
    switch(direction) {
      case FORWARD:
        drive(speed, speed);
        break;

      case BACKWARD:
        drive(-speed, -speed);
        break;
    }

    return;
  }
};

void Robot::drive(int fl, int fr) {
  if(type == MECANUM) return;

  this->fl.run(fl);
  this->fr.run(fr);
}

void Robot::drive(int fl, int fr, int br, int bl) {
  if(type == TWO_WHEEL) return;

  this->fl.run(fl);
  this->fr.run(fr);
  this->br.run(br);
  this->bl.run(bl);
}

void Robot::drive(moveDirection direction, int speed, int duration) {
  drive(direction, speed);
  delay(duration);
  stop();
}

void Robot::drive(int fl, int fr, int duration) {
  drive(fl, fr);
  delay(duration);
  stop();
}

void Robot::drive(int fl, int fr, int br, int bl, int duration) {
  drive(fl, fr, br, bl);
  delay(duration);
  stop();
}

void Robot::turn(turnDirection direction, int speed) {
  if(type == MECANUM) {
    switch(direction) {
      case CW:
        drive(speed, -speed, -speed, speed);
        break;

      case CCW:
        drive(speed, -speed, -speed, speed);
        break;
    }

    return;
  }

  if(type == TWO_WHEEL) {
    switch(direction) {
      case CW:
        drive(speed, -speed);
        break;

      case CCW:
        drive(-speed, speed);
        break;
    }

    return;
  }
};

void Robot::turn(turnDirection direction, int speed, int duration) {
  turn(direction, speed);
  delay(duration);
  stop();
};