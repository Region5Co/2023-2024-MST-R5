#include "StateMachine.h"
#include "State.hpp"
#include "Robot.h"


StateMachine::StateMachine(Robot *r){
    this->robot = r;
}

State* StateMachine::getState(){
    return this->state;
}

void StateMachine::init(State *s)
{
    state = s;
    Serial.println("State Assigned");
    delay(100);
}

void StateMachine::setState(State* s){
    this->state = s;
}

void StateMachine::run(){
    state->execute();
    Serial.println("State Executing: "+state->name);
    delay(100);
}   

void StateMachine::transition(int trigger){
    state->end();
    Serial.println("Moving to State: "+state->nodes[trigger+1]->next_state->name);
    setState((*(state->nodes[trigger])).next_state);
    state->init(robot);
}
