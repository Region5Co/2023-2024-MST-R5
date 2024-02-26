#include <Arduino.h>

class Motor {
  friend class Robot;
  public:
    Motor(int pin1, int pin2, int en);
    Motor(int pin1, int pin2, int en, bool reversed);

    void init();

    void setSpeed(int speed);
    void setReversed(bool reversed);
    void stop();
    void run(int velocity);

  private:  
    Motor();
    int pin1;
    int pin2;
    int en;
    bool reversed;
};