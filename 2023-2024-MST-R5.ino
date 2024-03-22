#include "src/Adafruit_LSM6DS/Adafruit_LSM6DS3TRC.h"
#include <Adafruit_VL53L1X.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>


#include "Robot.h"
#include "IEEE_Pinout.h"
#include "HCSR04.h"
#include <Servo.h>


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
HCSR04 hc(HC_TRIGGER, HC_ECHO);
Servo myservo;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

Robot robot(fl, fr, br, bl);

void setup() {

  Serial.begin(115200);
  while(!Serial) delay(10);


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
  robot.init();
  //robot.addIMU(imu) //to be added

  myservo.attach(SERVO_PIN);
}

void loop() {
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

