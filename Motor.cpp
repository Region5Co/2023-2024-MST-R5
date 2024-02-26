#include "Motor.h"

Motor::Motor() {
  pin1 = -1;
  pin2 = -1;
  en = -1;
  reversed = false;
}

Motor::Motor(int pin1, int pin2, int en) {
  Motor(pin1, en, pin2, false);
}

Motor::Motor(int pin1, int pin2, int en, bool reversed) {
  this->pin1 = pin1;
  this->pin2 = pin2;
  this->en = en;
  this->reversed = reversed;
}

void Motor::init() {
  pinMode(pin1, OUTPUT);
  //pinMode(pin2, OUTPUT);
  pinMode(en, OUTPUT);
}

void Motor::setSpeed(int speed) {
  analogWrite(en, constrain(map(speed, 0, 100, 0, 255), 0, 255));
}

void Motor::setReversed(bool reversed) {
  this->reversed = reversed;
}

void Motor::stop() {
  setSpeed(0);
  digitalWrite(pin1, 0);
  digitalWrite(pin2, 0);
}

void Motor::run(int velocity) {
  if(velocity == 0) {
    stop();
    return;
  }

  if(velocity > 0) {
    setSpeed(velocity);
    digitalWrite(pin1, !reversed);
    digitalWrite(pin2, reversed);
    return;
  }

  if(velocity < 0) {
    setSpeed(velocity * -1);
    digitalWrite(pin1, reversed);
    digitalWrite(pin2, !reversed);
    return;
  }
} 