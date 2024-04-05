#include "Encoder.h"

//@brief Assigns interrupt and direction pins to an encoder class
//@param interruptPin is one of the encoder outputs tied to an interrupt pin on the Arduino
//@param directionPin is the other encoder output tied to a GPIO that tells the program the direction of rotation
//
Encoder::Encoder(int interruptPin, int directionPin){
  this->intPin = interruptPin;
  this->dirPin = directionPin;
}

//@brief Initializes control pins by turning respectives ports to outputs, additionally calculates the wheel circumference
//
void Encoder::init() {
  //attachInterrupt(digitalPinToInterrupt(this->intPin), interruptEncoder, RISING);
  pinMode(this->intPin, INPUT_PULLUP);
  pinMode(this->dirPin, INPUT);
  wheelCircum = 2 * PI * WHEEL_RADIUS;
}

//@brief Gets the raw encoder count from the encoder
//
int Encoder::getEncCount() {
  return this->encCount;
}
int Encoder::getCurMoveEncCount(){
  return this->curMoveEncCount;
}

//@brief Gets the encoder interrupt pin number
//
int Encoder::getEncIntPin(){
  return this->intPin;
}

//@brief Gets the total distance calculated from the number of encoder rotations (in inches)
//
float Encoder::getEncDist(){
  return float(this->encCount)/358.0*5.874778;
}

//Not tested...
float Encoder::getRPM(){
  RPM = ((currTime-lastTime)*60000000)/358;
  return this->RPM;
}

//@brief Increments or decrements the total encoder count based on the direction of rotation
//
void Encoder::incEncCount(){
  this->currTime = millis();
  if (digitalRead(this->dirPin) > 0){
    this->encCount +=1;
    this->curMoveEncCount +=1;
  } else {
    this->encCount -=1;
    this->curMoveEncCount -=1;
  }
  this->lastTime = this->currTime;
}

void Encoder::clearEncCount(){
  this->curMoveEncCount = 0;
}

float Encoder::getCurMoveEncDist(){
  return float(this->curMoveEncCount)/358.0*5.874778;
}
