#include "Robot.h"
#include "IEEE_Pinout.h"


static Motor fr(M_FRONT_RIGHT_PWM, M_FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE);
static Motor fl(M_FRONT_LEFT_PWM, M_FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE);
static Motor br(M_BACK_RIGHT_PWM, M_BACK_RIGHT_DIR, BACK_MOTORS_ENABLE);
static Motor bl(M_BACK_LEFT_PWM, M_BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

#if ENCODERS_ENABLE
  static Encoder encFL(E_FRONT_LEFT_INT, E_FORNT_LEFT_DIR);
  static Encoder encFR(E_FRONT_RIGHT_INT, E_FRONT_RIGHT_DIR);
  static Encoder encBL(E_BACK_LEFT_INT, E_BACK_LEFT_DIR);
  static Encoder encBR(E_BACK_RIGHT_INT, E_BACK_RIGHT_DIR);

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
#endif

static Robot robot(&fl, &fr, &br, &bl);



void setup() {
  #if ENCODERS_ENABLE  
    //Attach Encoders to Motors
    fl.attachEncoder(&encFL);
    fr.attachEncoder(&encFR);
    bl.attachEncoder(&encBL);
    br.attachEncoder(&encBR);

    //Setup Interrupts
    attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
    attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
    attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
    attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);
  #endif

  robot.init();
  

}

void loop() {
  robot.drive(FORWARD, 70, 1500);
  robot.drive(BACKWARD, 70, 1500);
  robot.drive(LEFT, 70, 1500);
  robot.drive(RIGHT, 70, 1500);
  robot.turn(CW, 70, 1500);
  robot.turn(CCW, 70, 1500);
 
}

