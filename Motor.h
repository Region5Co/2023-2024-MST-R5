#include <Arduino.h>

class Motor {
  friend class Robot;
  public:
    Motor(int pwmPin, int dirPin, int enPin);
    Motor(int pwmPin, int dirPin, int enPin, bool reversed);

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