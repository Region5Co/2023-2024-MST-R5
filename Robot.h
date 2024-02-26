#ifndef Robot_H
#define Robot_H

#include <Arduino.h>
#include "Motor.h"
#include "types.h"

#define MECANUM drivetrain::mecanum
#define TWO_WHEEL drivetrain::twoWheel

#define FORWARD moveDirection::forward
#define BACKWARD moveDirection::backward
#define LEFT moveDirection::left
#define RIGHT moveDirection::right

#define CW turnDirection::cw
#define CCW turnDirection::ccw

class Robot {
  public:
    Robot(int m1p1, int m1p2, int m1EN, int m2p1, int m2p2, int m2EN, int m3p1, int m3p2, int m3EN, int m4p1, int m4p2, int m4EN);
    Robot(Motor fl, Motor fr, Motor br, Motor bl);
    Robot(int m1p1, int m1p2, int m1EN, int m2p1, int m2p2, int m2EN);
    Robot(Motor fl, Motor fr);

    void init();

    void reverseMotors(bool fl, bool fr);
    void reverseMotors(bool fl, bool fr, bool br, bool bl);

    void stop();


    void drive(moveDirection direction, int speed);
    void drive(int fl, int fr);
    void drive(int fl, int fr, int br, int bl);

    void drive(moveDirection direction, int speed, int duration);
    void drive(int fl, int fr, int duration);
    void drive(int fl, int fr, int br, int bl, int duration);

    void turn(turnDirection direction, int speed);
    void turn(turnDirection direction, int speed, int duration);

  private:
    drivetrain type;

    Motor fl;
    Motor fr;
    Motor br;
    Motor bl;
};

#endif