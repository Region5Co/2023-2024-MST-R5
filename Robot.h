#ifndef Robot_H
#define Robot_H

#include <Arduino.h>
#include "Motor.h"
#include "types.h"
#include "Gyro.h"

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
    Robot(Motor fl, Motor fr, Motor br, Motor bl);

    void init();

    void reverseMotors(bool fl, bool fr, bool br, bool bl);

    void stop();

    void drive(moveDirection direction, int speed);
    void drive(int fl, int fr, int br, int bl);
    void drive(moveDirection direction, int speed, int duration);
    void drive(int fl, int fr, int br, int bl, int duration);
    void addIMU(Gyro*);
    void turn(turnDirection direction, int speed);
    void turn(turnDirection direction, int speed, int duration);
    float getAngle();
  private:
    Motor fl;
    Motor fr;
    Motor br;
    Motor bl;
    Gyro* imu;
};

#endif