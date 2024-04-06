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
  float angle,x,y,ft, left_right,reset_angle;
};
traverse_node A_to_D{45,2,0,8.4853,1,-135};
traverse_node B_to_G{-45,6,0,8.4853,0,135};
traverse_node C_to_A{-16.93,8,2,6.3246,0,163.07};
traverse_node D_to_H{-28.07,8,6,8.9443,0,151.93};
traverse_node E_to_C{-16.93,6,8,6.3246,0,163.07};
traverse_node F_to_B{-28.07,2,8,8.9443,0,151.93};
traverse_node G_to_E{-16.93,0,6,6.3246,0,163.07};
traverse_node H_to_F{-73.07,0,2,6.3246,0,106.93};

static const traverse_node Travese_Nodes[]={A_to_D,
      D_to_H,H_to_F,F_to_B,B_to_G,G_to_E,E_to_C};
/**********END OF TRAVERSE****************/


float desired_angle=0.0;
int i=0;
int movement =0;
float kp=3.1;
double speed=75;
float dist=0;
float error=0.0;
void setup() {
 
  Serial.begin(115200);
  Serial.println("In Setup");
  delay(100);

   
  gyro = Gyro(false,true);
  robot.addIMU(&gyro);
  robot.init();
  an=gyro.update();
  //myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);
  // an=180;
  int j=0;
  while(j<80){
    robot.drive(speed*0.7,0,(robot.getAngle()-an)*kp);
    delay(10);
    j++;
  }

}

void loop() {
  double n = gyro.update();
  Serial.print("Anlge: ");
  Serial.println(n);
  Serial.print("Desired: ");
  Serial.println(desired_angle);
  delay(250);
  switch(movement){

    default:
    case 0:
      //rotate from ass to wall
      desired_angle = Travese_Nodes[i].angle;
      error= gyro.update()-desired_angle;
      if(abs(error)>TOLERANCE){
        robot.drive(0,0,error*kp);
      }else{
        movement=2;
        robot.stop();
      }
      break;
    case 1:
      //drive straight
      error= gyro.update()-desired_angle;
      if(dist<Travese_Nodes[i].ft){
        robot.drive(speed,0,error*kp);
        dist+=0.001;
      }else{
        movement=0;
        i = (i>=MAX_T_INDEX)? 0: (i+1);
        robot.stop();
      }
    case 2:
      //re-orientate
      desired_angle = Travese_Nodes[i].reset_angle;
      error= gyro.update()-desired_angle;
      if(abs(error)>TOLERANCE){
        robot.drive(0,0,error*kp);
        gyro.reset();
      }else{
        movement=0;
        robot.stop();
      }
    break;
    

  }
  //robot.turn(75,180);
  
  
  //delay(2000);
  //turn 45 degrees CW
  // int current_pos = encoder.getEncDist();
  // int goal = Travese_Nodes[i].ft - current_pos;
  // Serial.println(encoder.getEncDist());
  // i=0;
  // while(encoder.getEncDist()<goal){
  //   robot.drive(FORWARD, 60, 1000);
  //   //checkObstruction(forward, 200,Travese_Nodes[i].left_right);
  // }
  // i= (i>=MAX_T_INDEX)? 0: i+1;

}


