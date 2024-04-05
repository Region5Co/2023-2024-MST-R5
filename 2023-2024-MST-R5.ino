
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

#include "IEEE_Pinout.h"
#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"


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

#define MAX_T_INDEX 7
//IMU
Gyro gyro(false,true);
//Robot Initialization
static Robot robot(&fl, &fr, &br, &bl);

//State Machine Initializtion
StateMachine machina(&robot);
 
//State Pointers
InitState i;

//Node array init
static State::trans_node init_nodes[MAX_NODES];
static State::trans_node traverse_nodes[MAX_NODES];
static State::trans_node orient_nodes[MAX_NODES];

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

void triggers(void*);
void updater(void* pvParamaters);
void run(void* pvParameters);



SemaphoreHandle_t Semaphore_n_angle;
float n_angle=0.0;

SemaphoreHandle_t Semaphore_T_Index;
int traverse_index=0;

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
  
  i=InitState();
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

  Semaphore_T_Index = xSemaphoreCreateMutex();
  Semaphore_n_angle = xSemaphoreCreateMutex();

  //robot init

  //Imu initialization
  if(!imu.begin_I2C(0x6B)){
    Serial.println("ERROR Initializing I2C for IMU");
  }
  /*Not using the accelerometer*/
  //imu.setAccelRange(ACCEL_RANGE);
  //imu.setAccelDataRate(ACCEL_RATE);
  imu.setGyroRange(GYRO_RANGE);
  imu.setGyroDataRate(GYRO_RATE);
  //have to check this portion
  //not using Accel.
  //imu.configInt1(false, false, true); // accelerometer DRDY on INT1 of the imu
  imu.configInt1(false, true, false); // gyro DRDY on INT1 of the imu

  
  if (!vl53.begin(0x29)) {
    Serial.print(F("Error on init"));
    Serial.println(vl53.vl_status);
  }
  if (!vl53.startRanging()) {
    Serial.print(F("sucks at ranging"));
    Serial.println(vl53.vl_status);
  }
  vl53.setTimingBudget(500);
  vl53.VL53L1X_SetDistanceMode(2);
  vl53.VL53L1X_SetROI(4, 16);


  Serial.println("Initializing robot");
  robot.addIMU(&gyro);
  robot.init();
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  machina.init(inti);


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


