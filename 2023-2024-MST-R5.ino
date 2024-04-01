
#include "Robot.h"
#include "IEEE_Pinout.h"
#include "Gyro.h"


//Motors
Motor fr(FRONT_RIGHT_PWM, FRONT_RIGHT_DIR, FRONT_MOTORS_ENABLE, false);
Motor fl(FRONT_LEFT_PWM, FRONT_LEFT_DIR, FRONT_MOTORS_ENABLE, false);
Motor br(BACK_RIGHT_PWM, BACK_RIGHT_DIR, BACK_MOTORS_ENABLE, false);
Motor bl(BACK_LEFT_PWM, BACK_LEFT_DIR, BACK_MOTORS_ENABLE, true);

//IMU
Gyro gyro;
Robot robot(fl, fr, br, bl);

void setup() {
  gyro = Gyro(false,true);
  Serial.begin(115200);
  while(!Serial) delay(10);
  gyro.init();
  gyro.calibrate();

  Serial.println("Initializing");
  robot.init();
  robot.addIMU(&gyro); //to be added
}

void loop() {
  gyro.update();
  Serial.println(robot.getAngle());
  delay(10);

  robot.drive(FORWARD, 100, 1500);
  robot.drive(BACKWARD, 100, 1500);
  robot.drive(LEFT, 100, 1500);
  robot.drive(RIGHT, 100, 1500);
  robot.turn(CW, 100, 1500);
  robot.turn(CCW, 100, 1500);

}

