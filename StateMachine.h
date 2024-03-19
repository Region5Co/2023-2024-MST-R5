#ifndef STATE_MACHINE
#define STATE_MACHINE

#include <Arduino.h>
#include "State.h"

class StateMachine {

public:
    StateMachine(Robot* r);
    State getState();
    void init(State* s);
    void run();
    void transition(int trigger);

    int scanTriggers();

private:
    void setState(State* s);

    
    Robot* robot;
    State* state;
};


#endif