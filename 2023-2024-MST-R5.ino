// #include <Arduino_FreeRTOS.h>
// #include <atomic.h>
// #include <event_groups.h>
// #include <FreeRTOSConfig.h>
// #include <FreeRTOSVariant.h>
// #include <list.h>
// #include <message_buffer.h>
// #include <mpu_wrappers.h>
// #include <portable.h>
// #include <portmacro.h>
// #include <projdefs.h>
// #include <queue.h>
// #include <semphr.h>
// #include <stack_macros.h>
// #include <stream_buffer.h>
// #include <task.h>
// #include <timers.h>

#include "Robot.h"

#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"

#define MAX_T_INDEX 7


/********ROBOT AND DEVICES*************/
//Devices
//Motors
Motor fl(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor bl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fr(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor br(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);
//IMU
Gyro gyro;
//Servo
Servo myservo;
//Ultrasonic
static Ultrasonic us;
//Robot Obj
Robot robot(&fl, &fr, &br, &bl);
/********END OF ROBOT INIT**********/



/*****State Machine and States INIT*****/
//State Machine Initializtion
static StateMachine machina(&robot);
//Trigger node array init
static State::trans_node init_nodes[MAX_NODES];
static State::trans_node traverse_nodes[MAX_NODES];
static State::trans_node orient_nodes[MAX_NODES];
/******END OF STATE MACHINE INIT*********/

double an;


/******Traverse Nodes Init***************/
typedef struct traverse_node{
  float angle,x,y,ft;
};
traverse_node A_to_D{45,2,0,8.4853};
traverse_node B_to_G{45,6,0,8.4853};
traverse_node C_to_A{71.5651,8,2,6.3246};
traverse_node D_to_H{63.4349,8,6,8.9443};
traverse_node E_to_C{71.5651,6,8,6.3246};
traverse_node F_to_B{63.4349,2,8,8.9443};
traverse_node G_to_E{71.5651,0,6,6.3246};
traverse_node H_to_F{18.4349,0,2,6.3246};

static const traverse_node Travese_Nodes[]={A_to_D,
      D_to_H,H_to_F,F_to_B,B_to_G,G_to_E,E_to_C};
/**********END OF TRAVERSE****************/




/*******RTOS INIT******************/
//Loops
// void triggers(void*);
// void updater(void* pvParamaters);
// void run(void* pvParameters);

//Semaphores
// SemaphoreHandle_t Semaphore_n_angle;
// float n_angle=0.0;
// SemaphoreHandle_t Semaphore_T_Index;
// int traverse_index=0;
/********END OF RTOS INIT***********/


double error1=0;
void setup() {
 
  #ifdef IEEE_SERIAL
   
  #endif
   Serial.begin(115200);
    Serial.println("In Setup");
    delay(100);


 
  
  gyro = Gyro(false,true);
  robot.addIMU(&gyro);
  robot.init();
 delay(100);
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);
  an = gyro.getGyroZ();
  #ifdef IEEE_SERIAL
  Serial.println(an);
     delay(100);
  #endif
}

void loop() {
  double n = gyro.update();

  
  
   double error=an-n;
 
    float k=-3.5;
    float k2 = 1;
    int speed = 50;
    //Serial.println(gyro.getGyroZ());
    robot.drive(speed,0,(error)*k);
    //delay(100);
    error1=error;
}
 
void updater(void*){
  //gyro.update();
}


void run(void*){

  //machina.run();  //Execute Current state execution  
}


void triggers(void*){
  while(1){
    State* curr_state = machina.getState();    //Grab current state from State Machine 
    int trigger = scanTriggers(curr_state);    //scan through triggers of current state
    if(trigger != -1){
      machina.transition(trigger);}
      //this means we're moving to next node
    //   if (curr_state->nodes[trigger]->next_state->name=="Orient"){
    //     if(xSemaphoreTake(Semaphore_T_Index,(TickType_t)2) == pdTRUE){
    //         traverse_index = (traverse_index>=MAX_T_INDEX)? 0: traverse_index+1;
    //         xSemaphoreGive(Semaphore_T_Index);
    //     }else{

    //     }
    //   }
    // }
    // if(xSemaphoreTake(Semaphore_n_angle,(TickType_t)2) == pdTRUE){

    //   xSemaphoreGive(Semaphore_n_angle);
    // }else{

    // }
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


