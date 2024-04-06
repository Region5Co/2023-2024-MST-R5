#include "Encoder.h"
#include "Robot.h"

#include "StateMachine.h"
#include "State.hpp"
#include "Triggers.h"
#include "States/Init.hpp"

#define MAX_T_INDEX 7
#define TOLERANCE 5

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


float base_angle=0.0;
float kp=2.1;
int stage = 0;
int a = 190; //distance from the right wall
int b = 190; //distance from wall were facing
int c = 275; //distance to hit the button?
double an;
double error1=0;


void setup() {
  fl.attachEncoder(&encFL);
  fr.attachEncoder(&encFR);
  bl.attachEncoder(&encBL);
  br.attachEncoder(&encBR);
 
  Serial.begin(115200);
  Serial.println("In Setup");
  delay(100);
   
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

  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

  myservo.attach(SERVO_PIN);

  float de =1;
  fl.decrease(0.97);
  fr.decrease(0.97);
  bl.decrease(0.97);
  br.decrease(de);
  base_angle = robot.getAngle();
  myservo.write(50);
}


void loop() {
  float ann = robot.getAngle();
  float error=ann-base_angle;
 
  switch(stage){ //make stage variable
    case 0:

      robot.drive_enc(70, 0, 50, 86);
      delay(10);
      stage =0;
      break;
      /*
      //Move right based on IR sensor distance from wall, a
      Serial.println("In stage 0: Moving Right");
      robot.drive(70, 0, 0, 50, 6);
      delay(1000);
      robot.turn(CW, 90.0, true);
      stage++;
      robot.stop();
      delay(500);*/

    case 1:
      Serial.println("In stage 1: driving towards right wall");
      if(revisedDist(vl53.distance()) > a){
        robot.drive(60,0,0,50,0.25);
      }else{
        stage++;
        delay(1000);
        robot.turn(CCW, 90.0, true);
        robot.stop();
        delay(500);
      }
      break;

    case 2:
      //Drive forward based on IR sensor distance from front wall, b
      Serial.println("In stage 2: Driving Forward to front wall");
      if(revisedDist(vl53.distance()) > b){
        Serial.println(error);
        robot.drive(60,0,0,50,1);
      }else{
        stage++;
        delay(1000);
        robot.turn(CW, 90.0, true);
        robot.stop();
        delay(1000);
      }
      break;

    case 3:
      Serial.println("In stage 3: Drive away from right wall");
      if(revisedDist(vl53.distance()) < c){
        robot.drive(-60,0,0,50,0.25);
      }else{
        stage++;
        delay(1000);
        robot.turn(CCW, 90.0, true);
        robot.stop();
        delay(1000);
      }
      break;

    case 4: 
      //Drive forward to push button
      Serial.println("In stage 3: Drive forward");
      robot.drive(100,0,0,50,6);
      stage++;
      robot.stop();
      delay(1000);
      break;

    case 5: 
      //Drive backward away from button
      Serial.println("In stage 4: Drive Backwards");
      //drive backwards
      robot.drive(-100,0,0,50,4);
      robot.stop();
      delay(1000);
      stage++;
      break;

    case 6:
      //Rotate 180 to resume loop
      Serial.println("In stage 5: Rotate 180");
      robot.turn(CW, 180.0, true);
      delay(1000);
      stage=0;
      break;

    default:
      break;
    }
    delay(10);
  }
  


//OBSELETE

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
