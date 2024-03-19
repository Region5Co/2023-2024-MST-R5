
#include "../States/Fade.h"

int FadeState::init(Robot*r){
    return 0;
}

int FadeState::execute(){
    if(i>=255){
        j=-1;
    }else if(i<=0){
        j=1;
    }
    i+=j;
    analogWrite(LED_BUILTIN, i);
    delay(1);
    return 0;
}

int FadeState::end(){
    return 0;
}
