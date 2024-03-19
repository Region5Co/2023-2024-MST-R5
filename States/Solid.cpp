
#include "../States/Solid.h"

int SolidState::init(Robot*r){
    return 0;
}

int SolidState::execute(){
    digitalWrite(LED_BUILTIN,HIGH);
    return 0;
}

int SolidState::end(){
    return 0;
}
