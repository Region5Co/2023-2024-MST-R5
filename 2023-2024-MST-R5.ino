#include <Arduino_FreeRTOS.h>
#include <atomic.h>
#include <event_groups.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
#include <list.h>
#include <message_buffer.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <stack_macros.h>
#include <stream_buffer.h>
#include <task.h>
#include <timers.h>

#include "IEEE_Pinout.h"
#include "Robot.h"
#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/FadeState.hpp"
#include "States/Blink.hpp"
#include "States/Solid.hpp"

//Motor Initializations
Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

//Robot Initialization
Robot robot(fl, fr, br, bl);

//State Machine Initializtion
StateMachine machina(&robot);
 
//State Pointers
State* fade;

void triggers(void*);
void updater(void* pvParamaters);
void run(void* pvParameters);

void setup() {

  #ifdef IEEE_SERIAL
    Serial.begin(115200);
    Serial.println("In Setup");
  #endif
  
  xTaskCreate(
    updater
    , "Update Loop" // A name just for humans
    , 128      // This stack size can be checked and adjusted by reading the Stack Highwater
    , NULL
    , 2        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    , NULL );
   xTaskCreate(
    triggers
    , "Trigger Loop" // A name just for humans
    , 64      // This stack size can be checked and adjusted by reading the Stack Highwater
    , NULL
    , 3        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    , NULL );
    xTaskCreate(
    run
    , "Run Loop" // A name just for humans
    , 128      // This stack size can be checked and adjusted by reading the Stack Highwater
    , NULL
    , 3        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    , NULL );
 
  #ifdef IEEE_SERIAL
    Serial.println("Leaving setup");
  #endif

  //robot init
  robot.init();
  //Initialize State Machine
  machina.init(fade);

}

void loop() {
  
  #ifdef IEEE_SERIAL
    Serial.println("Trigger: "+(String)trigger);
  #endif
}
 
void updater(void*){

}


void run(void*){
  machina.run();  //Execute Current state execution  
}


void triggers(void*){
  State* curr_state = machina.getState();    //Grab current state from State Machine 
  int trigger = scanTriggers(curr_state);    //scan through triggers of current state
  if(trigger != -1){
    machina.transition(trigger);
  }
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
