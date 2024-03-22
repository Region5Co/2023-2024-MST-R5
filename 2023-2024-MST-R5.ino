#include <Adafruit_VL53L1X.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>


#include "Robot.h"
#include "IEEE_Pinout.h"
#include "HCSR04.h"
#include <Servo.h>


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
  while (!Serial) delay(10);
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
  Serial.println("Initializing robot");
  robot.init();

  myservo.attach(SERVO_PIN);
}

void loop() {
  //testUltra();
  //servoTest();
  int16_t distance;

  if (vl53.dataReady()) {
    distance = vl53.distance();
    Serial.println(distance);
  }
  //square();
}

void testUltra() {
  Serial.println(hc.dist());
}

void square() {
  robot.drive(FORWARD, 100, 1000);
  robot.drive(RIGHT, 100, 1000);
  robot.drive(BACKWARD, 100, 1000);
  robot.drive(LEFT, 100, 1000);
}

void servoTest() {
  myservo.write(0);
  delay(1500);
  myservo.write(180);
  delay(750);
}

void moveUntilLt(moveDirection dir, int targetDist) {
  while (hc.dist() > targetDist) {
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilGt(moveDirection dir, int targetDist) {
  while (hc.dist() < targetDist) {
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}

void moveUntilWithServo(moveDirection dir, int targetDist, bool isLt) { // 90 degree rotation on this lad lmao
  if (dir == FORWARD) {
    myservo.write(180);
  } else if (dir == RIGHT) {
    myservo.write(0);
  }
  delay(15);
  if (isLt) {
    moveUntilLt(dir, targetDist);
  } else {
    moveUntilGt(dir, targetDist);
  }
}