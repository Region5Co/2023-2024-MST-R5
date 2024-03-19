
#include "IEEE_Pinout.h"
#include "Robot.h"
#include "StateMachine.h"
#include "State.h"
#include "Triggers.h"
#include "./States/Fade.h"
#include "./States/Blink.h"
#include "States/Solid.h"

Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

Robot robot(fl, fr, br, bl);


StateMachine machina(&robot);

//States
FadeState fade;
SolidState solid;
BlinkState blink;


//Triggers
Trigger button_0 = b0;
Trigger button_1 = b0;

//Nodes
State::trans_node fade_to_solid = {&solid, &button_0};
State::trans_node solid_to_fade = {&fade, &button_1};
State::trans_node solid_to_blink = {&blink, &button_0};
State::trans_node blink_to_fade = {&fade, &button_0};
State::trans_node blink_to_solid = {&solid, &button_1};



/****************************************************************/


void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  robot.init();

  //add triggers
  fade.addNode(fade_to_solid);

  solid.addNode(solid_to_blink);
  solid.addNode(solid_to_fade);

  blink.addNode(blink_to_fade);
  blink.addNode(blink_to_solid);
  machina.init(&fade);
}

void loop() {
  machina.run();
  int trigger = machina.scanTriggers();
  if(trigger != -1){
    machina.transition(trigger);
  }
 
}

