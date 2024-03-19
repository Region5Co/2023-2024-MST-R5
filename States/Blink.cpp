
#include "../States/Blink.h"

int BlinkState::init(Robot*r){
    return 0;
}

int BlinkState::execute(){
    digitalWrite(LED_BUILTIN, i);
    j++;
    if (j >= 200){
        if(i==0){i=1;}
        else {i=1;}
        j=0;
    }
    delay(10);
    return 0;
}

int BlinkState::end(){
    return 0;
}
