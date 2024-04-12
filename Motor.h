#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Encoder.h"
#define MAX_RUN 100
class Motor {
  friend class Robot;
  public:
    Motor(int pwm, int dir, int en, bool rev);

    void init();

    void setSpeed(int speed);
    void setReversed(bool reversed);
    void stop();
    void run(int velocity);
    void decrease(float);

    void attachEncoder(Encoder* en);
    Encoder* getEncoder();
    float getAngularVelocity();
  
  private:  
    Motor();
    int pwmPin;
    int dirPin;
    int enPin;
    bool reversed;
    float d=0.0;
    Encoder* encoder;
};

#endif