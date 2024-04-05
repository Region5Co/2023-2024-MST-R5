#include "Encoder.h"
#include "Robot.h"
#include "IEEE_Pinout.h"
#include <Servo.h>
//#include <RangeSensor.h>
#include "Adafruit_VL53L1X.h"
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>
#include "src/Adafruit_LSM6DS/Adafruit_LSM6DS3TRC.h"

#define MAX_T_INDEX 7

#if !defined(ACCEL_RANGE) && !defined(ACCEL_RATE) && IEEE_ACCEL
#define ACCEL_RANGE LSM6DS_ACCEL_RANGE_16_G
#define ACCEL_RATE  LSM6DS_RATE_1_66K_HZ
#endif

#if !defined(GYRO_RANGE) && !defined(GYRO_RATE)
#define GYRO_RANGE  LSM6DS_GYRO_RANGE_500_DPS
#define GYRO_RATE   LSM6DS_RATE_1_66K_HZ
#endif

Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);
Servo myservo;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(21, 20);
Adafruit_LSM6DS3TRC imu;

//IMU
Gyro gyro(false,true);


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

Robot robot(fl, fr, br, bl);

void setup() {
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

  attachInterrupt(digitalPinToInterrupt(encFL.getEncIntPin()), interruptEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(encFR.getEncIntPin()), interruptEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(encBL.getEncIntPin()), interruptEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(encBR.getEncIntPin()), interruptEncoderBR, RISING);

  myservo.attach(SERVO_PIN);
  Serial.begin(9600);
}

int stage = 0;
  int a = 150; //distance from the right wall
  int b = 100; //distance from wall were facing
  int c = 305; //distance to hit the button?
void loop() {
  /*robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);*/

  switch(stage){ //make stage variable
    case 0: //need to get data from ultrasonic sensor (this is psuedocode)
    {
      /*
      robot.drive(FORWARD, 100, 500);
      Serial.println(robot.Get_Y_Pos());
      robot.drive(BACKWARD, 100, 500);
      Serial.println(robot.Get_Y_Pos());
      */   

      
      Serial.println("In stage 0");
      delay(2000);
      moveUntilWithServo(RIGHT, a, true);
      delay(1000);
      

      /*
      robot.drive(FORWARD, 70, 100);
      
      Serial.print("Encoder: ");
      Serial.print(encFL.getEncDist());
      Serial.print(" ");
      Serial.print(encFR.getEncDist());
      Serial.print(" ");
      Serial.print(encBL.getEncDist());
      Serial.print(" ");
      Serial.print(encBR.getEncDist());
      Serial.println(" ");
      */

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
      delay(2000);
      moveUntilWithServo(LEFT, c, false);
      delay(1000);

      stage = 3;
      break;
    }
    case 3: 
    {
      Serial.println("In stage 3");
      delay(1000);
      robot.drive(FORWARD, 100, 1500);
      delay(1000);

      stage = 4;
      break;
    }
    case 4: 
    {
      Serial.println("In stage 4");
      delay(1000);
      robot.drive(BACKWARD,100,1500);
      delay(500);
      robot.turn(CW,70,1000);
      delay(500);

      stage = 0;
      break;
      /* here would just need to back up a small bit*/
       
    }
  }
}


void square() {
  Serial.println("Forward");
  robot.drive(FORWARD, 100, 1000);
  Serial.println("Right");
  robot.drive(RIGHT, 100, 1000);
  Serial.println("Backward");
  robot.drive(BACKWARD, 100, 1000);
  Serial.println("Left");
  robot.drive(LEFT, 100, 1000);
}

void servoTest() {
  myservo.write(20);
  delay(1500);
  myservo.write(150);
  delay(750);
}

void moveUntilLt(moveDirection dir, int targetDist) {
  /*
  while (vl53.distance() == -1){
    Serial.println(vl53.distance());
    Serial.print("stuck in -1");
    robot.drive(dir, 100);
    delay(50);
  }*/

  while (revisedDist(vl53.distance()) > targetDist) {
    robot.drive(dir, 70);
    delay(10);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {
  while (revisedDist(vl53.distance()) < targetDist) {
    robot.drive(dir, 70);
    delay(10);
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
