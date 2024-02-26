#include "chrono"
#include "mecanum_driver.h"

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

void setup(){
  drive = new robot::robot_4_wheels(pinList);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
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

      break;
  }
}