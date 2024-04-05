
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

#include "Robot.h"

#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"

#if !defined(ACCEL_RANGE) && !defined(ACCEL_RATE) && IEEE_ACCEL
#define ACCEL_RANGE LSM6DS_ACCEL_RANGE_16_G
#define ACCEL_RATE  LSM6DS_RATE_1_66K_HZ
#endif

#if !defined(GYRO_RANGE) && !defined(GYRO_RATE)
#define GYRO_RANGE  LSM6DS_GYRO_RANGE_500_DPS
#define GYRO_RATE   LSM6DS_RATE_1_66K_HZ
#endif

#define MAX_T_INDEX 7
//Devices
Adafruit_LSM6DS3TRC imu;

//Motors
Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

//IMU
Gyro gyro(false,true);
//Robot Initialization
Servo myservo;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

Robot robot(fl, fr, br, bl);

/*****State Machine and States INIT*****/
//State Machine Initializtion
StateMachine machina(&robot);
 
//State Pointers
State* inti;
InitState i;

//Node array init
static State::trans_node init_nodes[MAX_NODES];
static State::trans_node traverse_nodes[MAX_NODES];
static State::trans_node orient_nodes[MAX_NODES];
/******END OF STATE MACHINE INIT*********/



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
void triggers(void*);
void updater(void* pvParamaters);
void run(void* pvParameters);

//Semaphores
SemaphoreHandle_t Semaphore_n_angle;
float n_angle=0.0;
SemaphoreHandle_t Semaphore_T_Index;
int traverse_index=0;
/********END OF RTOS INIT***********/



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


 
  #ifdef IEEE_SERIAL
    Serial.begin(115200);
    Serial.println("In Setup");
  #endif
  
  //States
  static InitState init_s;

  init_s=InitState();

  xTaskCreate(updater, "Update Loop", 64, NULL, 2, NULL);
  xTaskCreate(triggers, "Trigger Loop", 128, NULL, 3, NULL);
  xTaskCreate(run, "Run Loop", 128, NULL, 3, NULL);
  
  
  Semaphore_T_Index = xSemaphoreCreateMutex();
  Semaphore_n_angle = xSemaphoreCreateMutex();
  
 
  robot.addIMU(&gyro);
  robot.init();
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  machina.init(&init_s);
  
  #ifdef IEEE_SERIAL
    Serial.println("Leaving setup");
  #endif



}

void loop() {
}
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
  while(1){
    State* curr_state = machina.getState();    //Grab current state from State Machine 
    int trigger = scanTriggers(curr_state);    //scan through triggers of current state
    if(trigger != -1){
      machina.transition(trigger);
      //this means we're moving to next node
      if (curr_state->nodes[trigger]->next_state->name=="Orient"){
        if(xSemaphoreTake(Semaphore_T_Index,(TickType_t)2) == pdTRUE){
            traverse_index = (traverse_index>=MAX_T_INDEX)? 0: traverse_index+1;
            xSemaphoreGive(Semaphore_T_Index);
        }else{

        }
      }
    }
    if(xSemaphoreTake(Semaphore_n_angle,(TickType_t)2) == pdTRUE){

      xSemaphoreGive(Semaphore_n_angle);
    }else{

    }
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


