#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic{

public:
    Ultrasonic(int n);
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