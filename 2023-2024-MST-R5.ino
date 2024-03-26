#include "Robot.h"
#include "IEEE_Pinout.h"


static Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE);
static Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE);
static Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE);
static Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

static Encoder encFL(2, 7);
static Encoder encFR(3, 8);
static Encoder encBL(18, 9);
static Encoder encBR(19, 10);

static Robot robot(&fl, &fr, &br, &bl);



void interruptEncoderFL(){
  encFL.incEncCount();
}
void interruptEncoderFR(){
  encFR.incEncCount();
}
void interruptEncoderBL(){
  encBL.incEncCount();
}
void interruptEncoderBR(){
  encBR.incEncCount();
}

void setup() {
    
  fl.attachEncoder(&encFL);
  fr.attachEncoder(&encFR);
  bl.attachEncoder(&encBL);
  br.attachEncoder(&encBR);

  robot.init();
  
  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

}

void loop() {
  robot.drive(FORWARD, 70, 1500);
  robot.drive(BACKWARD, 70, 1500);
  robot.drive(LEFT, 70, 1500);
  robot.drive(RIGHT, 70, 1500);
  robot.turn(CW, 70, 1500);
  robot.turn(CCW, 70, 1500);
 
}

