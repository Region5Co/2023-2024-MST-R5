#include "StateMachine.h"
#include "State.h"
#include "Robot.h"

StateMachine::StateMachine(Robot *r){
    this->robot = r;
}

State StateMachine::getState(){
    return State();
}

void StateMachine::setState(State* s){
    this->state = s;
}

void StateMachine::run(){
    state->execute();
}   

void StateMachine::transition(int trigger){
    state->end();
    setState(state->nodes[trigger].next_state);
    state->init(robot);
}

int StateMachine::scanTriggers(){
    bool multi_trigger = false,
        triggered=false;

    for(short i=0; i < sizeof(state->nodes); i++){
        State::trans_node node = state->nodes[i];
        bool temp_trigger = triggered;
        triggered = *node.trigger; //this line runs the function
        if(triggered && temp_trigger){
            multi_trigger = true;
        }
        
    }
    //return triggered XOR multi_trigger
    return (!triggered && multi_trigger) || (triggered && !multi_trigger);
}