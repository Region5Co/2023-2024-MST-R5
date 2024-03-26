#include "Encoder.h"
#include "Arduino.h"

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
  wheelCircum = 2*3.14*0.935;
}

//@brief Gets the raw encoder count from the encoder
//
int Encoder::getEncCount() {
  return this->encCount;
}

//@brief Gets the encoder interrupt pin number
//
int Encoder::getEncIntPin(){
  return this->intPin;
}

//@brief Gets the total distance calculated from the number of encoder rotations (in inches)
//
int Encoder::getEncDist(){
  numRev = float(encCount)/358;
  return numRev*wheelCircum;
}

//Not tested...
float Encoder::getRPM(){
  return ((currTime-lastTime)*60000000)/358;
}

//@brief Increments or decrements the total encoder count based on the direction of rotation
//
void Encoder::incEncCount(){
  currTime = millis();
  if (digitalRead(this->dirPin) > 0){
    encCount +=1;
  } else {
    encCount -=1;
  }
  lastTime = currTime;
}
