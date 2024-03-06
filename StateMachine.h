#ifndef STATE_MACHINE
#define STATE_MACHINE

#include <Arduino.h>
#include "State.h"

class StateMachine {

public:
    StateMachine();
    State getState();
private:
    void setState(State s);
    
    State state;
};


#endif