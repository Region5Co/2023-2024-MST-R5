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

#define Kp 0.9

class Robot {
  public:
    Robot(Motor fl, Motor fr, Motor br, Motor bl);

    void init();

    void reverseMotors(bool fl, bool fr, bool br, bool bl);

    void stop();

    void drive(moveDirection direction, int speed);
    void drive(int fl, int fr, int br, int bl);
    void drive(moveDirection direction, int speed, int duration);
    void drive(int fl, int fr, int br, int bl, int duration);

    void turn(turnDirection direction, int speed);
    void turn(turnDirection direction, int speed, int duration);
    float Get_X_Pos();
    float Get_Y_Pos();
    void Update_Pos(moveDirection direction);
    void clearAllEncCount();

  private:
    float X_Pos;
    float Y_Pos;
    Motor fl;
    Motor fr;
    Motor br;
    Motor bl;

};

#endif