#include "IEEE_Pinout.h"
#include "Robot.h"
#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/FadeState.hpp"
#include "States/Blink.hpp"
#include "States/Solid.hpp"

//Motor Initializations

//Motors
Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

//IMU
Gyro gyro(false,true);
//Robot Initialization
Robot robot(fl, fr, br, bl);

//State Machine Initializtion
StateMachine machina(&robot);
 
//State Pointers
State* fade;
State* solid;
State* blink;

//Node array init
static State::trans_node fade_nodes[MAX_NODES];
static State::trans_node solid_nodes[MAX_NODES];
static State::trans_node blink_nodes[MAX_NODES];
 


void setup() {
 
  #ifdef IEEE_SERIAL
    Serial.begin(115200);
    Serial.println("In Setup");
  #endif
  
  //robot init
  robot.addIMU(&gyro); //to be added
  robot.init();

  //Setup pins for test state machine
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IEEE_B0, INPUT);
  pinMode(IEEE_B1, INPUT);
  
  /**Create static States**/
  static FadeState  f;
  static BlinkState b;
  static SolidState s;

	/** Create Nodes to link to states**/
  //Nodes for Fadestate
	static State::trans_node fade_to_solid  = {&s, (Trigger)b0};
  fade_nodes[0] = fade_to_solid;

  //Nodes for SolidState
	static State::trans_node solid_to_fade  = {&f, (Trigger)b1};
	static State::trans_node solid_to_blink = {&b, (Trigger)b0};
	solid_nodes[1] = solid_to_fade;
	solid_nodes[0] = solid_to_blink;

  //Nodes for BlinkState
	const State::trans_node blink_to_fade  = {&f, (Trigger)b0};
	const State::trans_node blink_to_solid = {&s, (Trigger)b1};
	blink_nodes[0] = blink_to_fade;
	blink_nodes[1] = blink_to_solid;

  //Re-initialize States with nodes
  f = FadeState(fade_nodes,0);
  b = BlinkState(blink_nodes,1);
  s = SolidState(solid_nodes,1);

  //Assign states to pointers
  fade  = &f;
  blink = &b;
  solid = &s;
 
  #ifdef IEEE_SERIAL
    Serial.println("Leaving setup");
  #endif
  delay(1000);
  
  //Initialize State Machine
  machina.init(&f);

}

void loop() {
  machina.run();  //Execute Current state execution  
  State* curr_state = machina.getState();    //Grab current state from State Machine 
  int trigger = scanTriggers(curr_state);    //scan through triggers of current state

  #ifdef IEEE_SERIAL
    Serial.println("Trigger: "+(String)trigger);
  #endif

  if(trigger != -1){
    machina.transition(trigger);
  }

  delay(50);
 
}

int scanTriggers(State* state){

  bool multi_trigger = false,
    triggered = false,
    temp_trigger = false;
  int _trigger = -1;
  
  #ifdef IEEE_SERIAL
    Serial.println("Name: "+state->name);
    Serial.println("Size: "+ (String)state->size);
  #endif

  for(int i=0; i <= state->size; i++){
    State::trans_node* node =  (*state).nodes[i];
    triggered = (*node).trigger(&robot);
    _trigger = (triggered)? i : -1;

    #ifdef IEEE_SERIAL
      Serial.println("Iteration: "+(String)i);
      Serial.println("Next State: " + (*node).next_state->name);
      delay(2500);
    #endif   
  }
  //return triggered XOR multi_trigger
  // bool _ret = ((!triggered && multi_trigger) || (triggered && !multi_trigger));
  // if(_ret || _trigger == -1){
  //     return -1;
  // }
  return _trigger;
}

