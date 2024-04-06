#include "Robot.h"

#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"

#define MAX_T_INDEX 7
#define TOLERANCE 5.0
#define DANGER_RANGE 150

/********ROBOT AND DEVICES*************/
//Devices
//Motors
Motor fl(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor bl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fr(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor br(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);


Encoder encFL(E_FRONT_LEFT_INT, E_FORNT_LEFT_DIR);
Encoder encFR(E_FRONT_RIGHT_INT, E_FRONT_RIGHT_DIR);
Encoder encBL(E_BACK_LEFT_INT, E_BACK_LEFT_DIR);
Encoder encBR(E_BACK_RIGHT_INT, E_BACK_RIGHT_DIR);


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

//Robot Obj
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(21, 20);

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
double dist=0.0;
float error=0.0;
void setup() {
 
  Serial.begin(115200);
  Serial.println("In Setup");
  delay(100);
  //us= Ultrasonic(0x29);

  fl.attachEncoder(&encFL);
  fr.attachEncoder(&encFR);
  bl.attachEncoder(&encBL);
  br.attachEncoder(&encBR);
   float de = 0.9;
  fl.decrease(de);
  fr.decrease(de);
  bl.decrease(de);
  br.decrease(1.0);
  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

  gyro = Gyro(false,true);
  robot.addIMU(&gyro);
  robot.init();
  an=gyro.update();
  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);
  myservo.write(50);

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

  // an=180;
  int j=0;
  robot.drive(65,0,0,50,3);
 
}

void loop() {
  double n = gyro.update();
  Serial.print("Anlge: ");
  Serial.println(n);
  Serial.print("Desired: ");
  Serial.println(desired_angle);
  //delay(250);
  switch(movement){
    default:
    case 0:
      Serial.println("Case 0");
      //rotate FROM ass to wall
      desired_angle = Travese_Nodes[i].angle;
      error= gyro.update()-desired_angle;
      Serial.print("Desired: ");
  Serial.println(desired_angle);
        if(error>0){robot.turn(CCW,(desired_angle),false);}
        else{robot.turn(CW,(desired_angle),false);}
      dist=0;
        movement++;
        robot.stop();
        target_dist = Travese_Nodes[i].ft;
      
      break;
    case 1:
      //drive straight
      Serial.println("Case 1");
      error= gyro.update()-desired_angle;
      if(dist<target_dist){
        //Avoid if needed
        if (revisedDist(vl53.distance()) < DANGER_RANGE ){
          //greater than 8 inches
          Serial.println("Danger Zone");
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
            dist=0;
            Travese_Nodes[i].reset_angle = (180-desired_angle) * ((Travese_Nodes[i].left_right)? -1:1);
            //end of go around charger
          }else if(Travese_Nodes[i].left_right){
            //turn -90 
              robot.turn(CCW,90,false);
            //straight 6''
              robot.drive(65,0,0,50,6);
            //drive 90
              robot.turn(CW,90,false);
            //straight 12'' //keep track of dist
              robot.drive(65,0,0,50,12);
              dist+=1;
            //drive 90
              robot.turn(CW,90,false);
            //straight 6''
              robot.drive(65,0,0,50,12);
            //turn -90
              robot.turn(CCW,90,false);
            //end of go around left
          }else{
            //turn 90 
              robot.turn(CW,90,false);
            //straight 6''
              robot.drive(65,0,0,50,6);
            //drive -90
              robot.turn(CCW,90,false);
            //straight 12'' //keep track of dist
              robot.drive(65,0,0,50,12);
              dist+=1;
            //drive -90
              robot.turn(CCW,90,false);
            //straight 6''
              robot.drive(65,0,0,50,12);
            //turn 90
              robot.turn(CW,90,false);
            //end of go around right
          }
          //end of avoidance
        }else{
          //drive inch by inch
          Serial.print("Driving: ");
          Serial.println(dist);
          robot.drive(70,0,0,10,1);
          dist+=0.0833;
        }
       //reached of target distance
      }else{
        movement++;
        dist=0;
        i = (i>=(MAX_T_INDEX-1))? 0: (i+1);
        robot.stop();
      }
      break;

    case 2:
      Serial.println("Case 2");
      //re-orientate aka rotate ass TO wall
      desired_angle = Travese_Nodes[i].reset_angle;
      error= gyro.update()-desired_angle;
       if(error>0){robot.turn(CW,(desired_angle),false);}
       else{robot.turn(CCW,abs(desired_angle),false);}
        gyro.reset();
        movement=0;
        robot.stop();
      
    break;

  }

}

int revisedDist(int dist)   {  
    if (dist==-1){
        dist=1000;
    }
    return dist;
}


