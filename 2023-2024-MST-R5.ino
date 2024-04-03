#include "src/Adafruit_LSM6DS/Adafruit_LSM6DS3TRC.h"
#include <Adafruit_VL53L1X.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>
#include <Servo.h>

#include "Robot.h"
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

//State Machine Initializtion
StateMachine machina(&robot);
 
//State Pointers
State* inti;
InitState i;

//Node array init
static State::trans_node init_nodes[MAX_NODES];
static State::trans_node traverse_nodes[MAX_NODES];
static State::trans_node orient_nodes[MAX_NODES];




void triggers(void*);
void updater(void* pvParamaters);
void run(void* pvParameters);

void setup() {
 
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

  //robot init

  //Imu initialization
  if(!imu.begin_I2C()){
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

  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init"));
    Serial.println(vl53.vl_status);
  }
  if (! vl53.startRanging()) {
    Serial.print(F("sucks at ranging"));
    Serial.println(vl53.vl_status);
  }
  vl53.setTimingBudget(500);
  vl53.VL53L1X_SetDistanceMode(2);
  vl53.VL53L1X_SetROI(4, 16);


  Serial.println("Initializing robot");
  robot.addIMU(*gyro);
  robot.init();
  myservo.attach(SERVO_PIN);
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
  float x = getGyroX(&imu);
  Serial.println(x);
  //robot.drive(FORWARD, 70, 1500);
  //robot.drive(BACKWARD, 70, 1500);
  //robot.drive(LEFT, 70, 1500);
  //robot.drive(RIGHT, 70, 1500);
  //robot.turn(CW, 70, 1500);
  //robot.turn(CCW, 70, 1500);
 
}

float getGyroX(Adafruit_LSM6DS3TRC* gyro){
  sensors_event_t e;
  gyro->getEvent(nullptr, &e, nullptr);
  return e.gyro.z;
}

