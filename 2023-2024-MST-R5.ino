#include "chrono"
#include "mecanum_driver.h"
#include <Ultrasonic.h>

#define R5_ROUND 1
#if R5_ROUND == 2
#define STATEMACHINE 2
#else
#define STATEMACHINE 1
#endif

int pinList[12] = {5,0,6,     //fRight 
                   7,0,8,     //fLeft
                   7,8,9,     //bRight
                   10,11,12}; //bLeft

robot::robot_4_wheels* drive;
int stage = 1;

void setup(){
  drive = new robot::robot_4_wheels(pinList);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      //ultrasonic.setDirection(right);
      break;
  }
}

void loop(){
  drive->move_right(50);
  delay(1000);
  drive->move_down(50);
  delay(1000);
  drive->move_left(50);
  delay(1000);
  drive->move_up(50);
  delay(1000);
  drive->halt_all();
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      switch (stage) {
        case 1: // hug the right wall
          // if (ultrasonic.distance() < 10) {
            drive->move_right(50);
          //} else { drive->halt_all(); stage = 2;}
          break;
        case 2: // go to the other end
          //ultrasonic.setDirection(forward);
          // if (ultrasonic.distance() < 10) {
            drive->move_up(50);
          //} else { drive->halt_all(); stage = 3;}
          break;
        case 3: // center robot boop button forward
          //ultrasonic.setDirection(left);
          // if (ultrasonic.distance() < 20) {
            drive->move_left(50);
          //} else { drive->move_up(50); delay(500); drive->move_down(30); delay(200); drive->halt_all(); stage = 4;}
          break;
        case 4: // hug the left wall
          //ultrasonic.setDirection(down);
          // if (ultrasonic.distance() < 10) {
            drive->move_down(50);
          //} else { drive->halt_all(); stage = 5;}
          break;
        case 5: // center robot boop button backward
          //ultrasonic.setDirection(right);
          // if (ultrasonic.distance() < 20) {
            drive->move_right(50);
          //} else { drive->move_down(50); delay(500); drive->move_up(50); delay(200); drive->halt_all(); stage = 6;}
        default:
          break;
      }
      break;
  }
}