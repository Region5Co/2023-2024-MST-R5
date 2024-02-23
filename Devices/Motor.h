#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor{

public:
    Motor(int n);
    void init();
    void setSpeed(int speed);
    int getSpeed();
    void stop();
    int getNumber();

private:
    int motor_no;
    int speed;

};

#endif