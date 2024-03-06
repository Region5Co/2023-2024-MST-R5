#include <Arduino.h>

class Motor {
  friend class Robot;
  public:
    Motor(int pwm, int dir, int en, bool rev);

    void init();

    void setSpeed(int speed);
    void setReversed(bool reversed);
    void stop();
    void run(int velocity);

  private:  
    Motor();
    int pwmPin;
    int dirPin;
    int enPin;
    bool reversed;
};