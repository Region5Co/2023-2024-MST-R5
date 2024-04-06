#include "Robot.h"

#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"

#define MAX_T_INDEX 7
#define TOLERANCE 5.0
#define DANGER_RANGE 200

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
//trig_bool true sin is for x false sin is for y
typedef struct traverse_node{
  float angle,x,y,ft, left_right,reset_angle;
  int trig_bool;
};
traverse_node A_to_D{45,2,.25,8.4853,1,-135,1};
traverse_node D_to_H{-28.07,7.75,6,8.9443,0,151.93,1};
traverse_node H_to_F{-73.07,0.25,2,6.3246,0,106.93,0};
traverse_node F_to_B{-28.07,2,7.75,8.9443,0,151.93,0};
traverse_node B_to_G{-45,6,.25,8.4853,0,135,1};
traverse_node G_to_E{-16.93,0.25,6,6.3246,0,163.07,0};
traverse_node E_to_C{-16.93,6,7.75,6.3246,0,163.07,0};
traverse_node C_to_A{-16.93,7.75,2,6.3246,0,163.07,1};


static traverse_node Travese_Nodes[]={A_to_D,
      D_to_H,H_to_F,F_to_B,B_to_G,G_to_E,E_to_C};
/**********END OF TRAVERSE****************/


float desired_angle=0.0;
int i=0;
int movement =0;
float kp=3.1;
double speed=95;
float target_dist;
double dist=-30;
float error=0.0;
void setup() {
 
  Serial.begin(115200);
  Serial.println("In Setup");
  delay(100);
  us= Ultrasonic(0x29);
   
  gyro = Gyro(false,true);
  robot.addIMU(&gyro);
  robot.init();
  an=gyro.update();
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);
  myservo.write(130);
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
      //rotate FROM ass to wall
      desired_angle = Travese_Nodes[i].angle;
      error= gyro.update()-desired_angle;
      if(abs(error)>TOLERANCE){
        robot.drive(0,0,error*kp);
      }else{
        movement++;
        robot.stop();
        target_dist = Travese_Nodes[i].ft;
      }
      break;
    case 1:
      //drive straight
      error= gyro.update()-desired_angle;
      if(dist<target_dist){
        if (revisedDist(us.getDistance()) < DANGER_RANGE ){
          //greater than 8 inches
          if((Travese_Nodes[i].ft-dist)<0.75){

            //rotate TO be ass to wall
            desired_angle = Travese_Nodes[i].reset_angle;
            while(abs(error)>TOLERANCE){
              error= gyro.update()-desired_angle;
              robot.drive(0,0,error*kp);
            }
            gyro.reset();
            robot.stop();
            
            //calaculate new theta
            float hyp = Travese_Nodes[i].ft-dist;
            float s = sin(Travese_Nodes[i].angle)*hyp;
            float c = cos(Travese_Nodes[i].angle)*hyp;
            float x = Travese_Nodes[i].x;
            float y = Travese_Nodes[i].y;
            x+= (Travese_Nodes[i].trig_bool)? s:c;
            y+= (Travese_Nodes[i].trig_bool)? c:s;
            desired_angle = atan(y/x) * ((Travese_Nodes[i].left_right)? 1:-1);

            //rotate to new desired angle  
            while(abs(error)>TOLERANCE){
              error= gyro.update()-desired_angle;
              robot.drive(0,0,error*kp);
            }
            gyro.reset();
            movement=0;
            robot.stop();
            target_dist = sqrt(pow((Travese_Nodes[i].x-x),2) + pow((Travese_Nodes[i].y-y),2));
            Travese_Nodes[i].reset_angle = (180-desired_angle) * ((Travese_Nodes[i].left_right)? -1:1);
            //end of go around charger
          }else if(Travese_Nodes[i].left_right){
            //turn -90 

            //straight 6''

            //drive 90

            //straight 12'' //keep track of dist

            //drive 90

            //straight 6''

            //turn -90

            //end of go around left
          }else{
            //turn 90 

            //straight 6''

            //drive -90

            //straight 12'' //keep track of dist

            //drive -90

            //straight 6''

            //turn 90
            //end of go around right
          }
          //end of avoidance
        }else{
          //drive inch by inch
          robot.drive(speed,0,error*kp);
          dist+=(1/12);
          delay(100);
        }
       //reached of target distance
      }else{
        movement++;
        dist=-30;
        i = (i>=MAX_T_INDEX)? 0: (i+1);
        robot.stop();
      }
      break;

    case 2:
      //re-orientate aka rotate ass TO wall
      desired_angle = Travese_Nodes[i].reset_angle;
      error= gyro.update()-desired_angle;
      if(abs(error)>TOLERANCE){
        robot.drive(0,0,error*kp);
      }else{
        gyro.reset();
        movement=0;
        robot.stop();
      }
    break;

  }

}

int revisedDist(int dist)   {  
    if (dist==-1){
        dist=1000;
    }
    return dist;
}


