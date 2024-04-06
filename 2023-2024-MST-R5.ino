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
#include "Encoder.h"
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
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(21, 20);
Adafruit_LSM6DS3TRC imu;

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

    //Imu initialization
  if(!imu.begin_I2C(0x6A)){
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

  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

  myservo.attach(SERVO_PIN);
  //Initialize State Machine
  //machina.init(&init_s);

  desired_angle = robot.getAngle();
}

int stage = 0;
  int a = 190; //distance from the right wall
  int b = 300; //distance from wall were facing
  int c = 200; //distance to hit the button?
void loop() {
  //delay(100);
  //error1=error;
  //delay(2000);
  //turn 45 degrees CW
  /*
  int current_pos = encoder.getEncDist();
  int goal = Travese_Nodes[i].ft - current_pos;
  Serial.println(encoder.getEncDist());
  i=0;
  while(encoder.getEncDist()<goal){
    robot.drive(FORWARD, 60, 1000);
    //checkObstruction(forward, 200,Travese_Nodes[i].left_right);
  }
  i= (i>=MAX_T_INDEX)? 0: i+1;
  */
  float ann = robot.getAngle();
  switch(stage){ //make stage variable
    case 0: //need to get data from ultrasonic sensor (this is psuedocode)
    {
      
      Serial.println("In stage 0");
      delay(100);
      moveUntilWithServo(RIGHT, a, true);
      delay(100);
      stage = 1;
      break;
    }
    case 1:
    {
      Serial.println("In stage 1");
      delay(2000);
      moveUntilWithServo(FORWARD, b, true);
      stage = 2;
      break;
    }
    case 2:
    {
      Serial.println("In stage 2");
      delay(100);
      moveUntilWithServo(LEFT, c, false);
      delay(100);
      stage = 3;
      break;
    }
    case 3: 
    {
      Serial.println("In stage 3");
      delay(100);
      //go full forward
      robot.drive(100,0,0);
      delay(100);
      stage = 4;
      break;
    }
    case 4: 
    {
      Serial.println("In stage 4");
      delay(1000);
      //drive backwards
      robot.drive(-100,0,0);
      delay(500);
      float kp=2.1;
      float error= ann-(desired_angle+180);
      //robot.turn(70, 180);
      robot.drive(0,0,(error)*kp);
      delay(500);
      stage = 0;
      break;
      /* here would just need to back up a small bit*/
       
    }
  }
}



void moveUntilLt(moveDirection dir, int targetDist) {
  an = gyro.update();
  while (revisedDist(vl53.distance()) > targetDist) {
    double n = gyro.update();
    double error=an-n;
    float k=-3.5;
    float k2 = 1;
    int speed = 50;
    //Serial.println(gyro.getGyroZ());
    if (dir == FORWARD) {
      Serial.print("I'm HERE");
      robot.drive(speed,0,(error)*k);
    } else {
      robot.drive(dir, 80);
    }
    //robot.drive(dir, 70);
    //delay(10);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {

  while (revisedDist(vl53.distance()) < targetDist) {
    double n = gyro.update();
  
    double error=an-n;
 
    float k=-3.5;
    float k2 = 1;
    int speed = 50;
    //Serial.println(gyro.getGyroZ());
    if (dir == FORWARD) {
      robot.drive(speed,0,(error)*k);
    } else {
      robot.drive(dir, 70);
    }
    //robot.drive(dir, 70);
    //delay(10);
  }
  robot.stop();
}

void moveUntilWithServo(moveDirection dir, int targetDist, bool isLt) { // 90 degree rotation on this lad lmao
  if (dir == FORWARD) {
    myservo.write(20);
  } else {
    myservo.write(150);
  }
  delay(2000);
  
  if (isLt) {
    moveUntilLt(dir, targetDist);
  } else {
    moveUntilGt(dir, targetDist);
  }
}

int revisedDist(int dist){  
    if (dist==-1){
        dist=1000;
    }
    return dist;
}
