#include "Drive/Drive.h"
#include "Devices/Lidar.h"

#define R5_ROUND 1
#if R5_ROUND == 2
#define STATEMACHINE 2
#else
#define STATEMACHINE 1
#endif

void setup(){
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      break;
  }
}

void loop(){
  switch(STATEMACHINE){
    case 2:
      break;
    default:
    case 1:
      break;
  }
}