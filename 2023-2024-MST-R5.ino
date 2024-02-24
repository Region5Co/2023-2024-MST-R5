#include "mecanum_driver.h"

#define R5_ROUND 1
#if R5_ROUND == 2
#define STATEMACHINE 2
#else
#define STATEMACHINE 1
#endif

int pinList[12] = {5,0,6,7,0,8,7,8,9,10,11,12};
//robot::motor test = robot::motor(1, 2, 3);

void setup(){
  robot::robot_4_wheels drive = robot::robot_4_wheels(pinList);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      //drive.move_right(255);
      break;
  }
}

void loop(){
  drive.move_right(100);
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:

      break;
  }
}