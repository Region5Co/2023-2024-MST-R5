#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Encoder.h"

class Motor {
  friend class Robot;
  public:
    Motor(int pwm, int dir, int en, bool rev);

    void init();

    void setSpeed(int speed);
    void setReversed(bool reversed);
    void stop();
    void run(int velocity);

    void attachEncoder(Encoder* en);
    Encoder* getEncoder();
    float getAngularVelocity();
  
  private:  
    Motor();
    int pwmPin;
    int dirPin;
    int enPin;
    bool reversed;
    Encoder* encoder;
};

#endif