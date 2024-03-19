#include "StateMachine.h"
#include "State.h"
#include "Robot.h"


StateMachine::StateMachine(Robot *r){
    this->robot = r;
}

State StateMachine::getState(){
    return State();
}

void StateMachine::init(State *s)
{
    this->state = s;
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
    int _trigger = -1;
    for(short i=0; i < sizeof(state->nodes); i++){
        State::trans_node node = state->nodes[i];
        bool temp_trigger = triggered;
        triggered = (*node.trigger)(robot); //this line runs the function
        if(triggered && temp_trigger){
            multi_trigger = true;
        }
        if(triggered){
            _trigger = i;
        }
    }
    //return triggered XOR multi_trigger
    bool _ret = ((!triggered && multi_trigger) || (triggered && !multi_trigger));
    if(_ret || _trigger == -1){
        return -1;
    }
    return _trigger;
}