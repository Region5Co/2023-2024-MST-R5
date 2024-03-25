#ifndef STATE_MACHINE
#define STATE_MACHINE

#include <Arduino.h>
#include "State.hpp"

class StateMachine {

public:
    StateMachine(Robot* r);
    void init(State* s);
    
    void run();
    void transition(int trigger);
    
    State* getState();
private:
    void setState(State* s);

    Robot* robot;
    State* state;
};


#endif