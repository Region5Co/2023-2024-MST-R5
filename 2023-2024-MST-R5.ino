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
#define TOLERANCE 5.0

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
  float angle,x,y,ft, left_right;
};
traverse_node A_to_D{45,2,0,8.4853,1};
traverse_node B_to_G{45,6,0,8.4853,0};
traverse_node C_to_A{71.5651,8,2,6.3246,0};
traverse_node D_to_H{116.6,8,6,8.9443,0};
traverse_node E_to_C{71.5651,6,8,6.3246,0};
traverse_node F_to_B{63.4349,2,8,8.9443,0};
traverse_node G_to_E{71.5651,0,6,6.3246,0};
traverse_node H_to_F{18.4349,0,2,6.3246,0};

static const traverse_node Travese_Nodes[]={A_to_D,
      D_to_H,H_to_F,F_to_B,B_to_G,G_to_E,E_to_C};
/**********END OF TRAVERSE****************/


float desired_angle=0.0;

int i=0;

double error1=0;
void setup() {
 
  #ifdef IEEE_SERIAL
   Serial.begin(115200);
    Serial.println("In Setup");
    delay(100);
  #endif
   
  gyro = Gyro(false,true);
  robot.addIMU(&gyro);
  robot.init();
  an=gyro.update();
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);
  an=180;

}

void loop() {
  double n = gyro.update();
  
  double error=n-an;
 
    float k=1.5;
    float k2 = 1;
    int speed = 50;
    //Serial.println(gyro.getGyroZ());
    robot.drive(0,0,(error)*k);
    //delay(100);
    error1=error;
    
    //robot.turn(75,180);
    
    
    //delay(2000);
    //turn 45 degrees CW
    //int current_pos = encoder.getEncDist();
    //int goal = Travese_Nodes[i].ft - current_pos;
    //Serial.println(encoder.getEncDist());
    // i=0;
    // while(encoder.getEncDist()<goal){
    //   robot.drive(FORWARD, 60, 1000);
    //   //checkObstruction(forward, 200,Travese_Nodes[i].left_right);
    // }
    i= (i>=MAX_T_INDEX)? 0: i+1;
  
}


