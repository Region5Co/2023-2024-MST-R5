#include "Motor.h"
#include "Arduino.h"

//@brief Motor class for "null" motor used for testing
//
Motor::Motor() {
  pwmPin = -1;
  dirPin = -1;
  reversed = true;
}


//@brief Assigns pins to a motor class
//@param pwmPin is the enable (A or B) pin on the DRV8835 breakoutboard
//@param dirPin is the phase (A or B) pin on the DRV8835 breakoutboard
//@param enPin is the Mode pin for A and B on the DRV8835 breakoutboard
//
Motor::Motor(int pwm, int dir, int en) {
  Motor(pwm, dir, en, false);
}

//@brief Assigns pins to a motor class with option for setting reversed polarity
//@param pwmPin is the enable (A or B) pin on the DRV8835 breakoutboard
//@param dirPin is the phase (A or B) pin on the DRV8835 breakoutboard
//@param enPin is the Mode pin for A and B on the DRV8835 breakoutboard
//@param reversed is to setup motor to run in reverse if it is mounted backwards
//
Motor::Motor(int pwm, int dir, int en, bool rev): 
  pwmPin(pwm),
  dirPin(dir),
  enPin(en),
  reversed(rev){}


//@brief Initializes control pins by turning respectives ports to outputs
//
void Motor::init() {
  pinMode(this->pwmPin, OUTPUT);
  pinMode(this->dirPin, OUTPUT);
  pinMode(this->enPin,  OUTPUT);
}

//@brief Sets speed of the pwmPin or enable of the motor
//@param speed An integer ranging from 0-100 that will be mapped and fitted to a 8-bit value (0-255)
//
void Motor::setSpeed(int speed) {
  analogWrite(pwmPin, constrain(map(speed, 0, 100, 0, 255), 0, 255));
}

//@brief Sets speed base polarity of the motor
//@param reversed true tells the motor to run in an opposing direction of its 'natural' rotation,
//                setting this to false allows the motor to spin in its 'natural' direction
//
void Motor::setReversed(bool reversed) {
  this->reversed = reversed;
}

//@brief Stops the motor by setting pins low and speed to 0
//NOTE: This function does not set enable pin low because the DRV8835's enable pin controls 2 motors not just the one
//
void Motor::stop() {
  setSpeed(0);
  digitalWrite(dirPin, 0);
}

//@brief Uses the inputed velocity to run the motor at a desired speed and corresponding dirrection
//@param velocity should be an integer from -100 to 100
//
void Motor::run(int velocity) {
  if(velocity == 0) {
    stop();
    return;
  }

  if(velocity > 0) {
    setSpeed(velocity);
    digitalWrite(dirPin, reversed);
    return;
  }

  if(velocity < 0) {
    setSpeed(velocity * -1);
    digitalWrite(dirPin, !reversed);
    return;
  }
} 