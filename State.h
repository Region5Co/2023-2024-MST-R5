#ifndef STATE_H
#define STATE_H

#define MAX_NODES 3

#include "Robot.h"

typedef bool (*Trigger)(); //in the parenthesis add any parameters to pass in all transitions

class State{
friend class StateMachine;
public:
    typedef struct trans_node{
        State* next_state;       // pointer to state to goto
        Trigger* trigger; //function pointer
    } trans_node;

    State(trans_node transitions[]);
    virtual int init(Robot* r);
    virtual int execute();
    virtual int end();
private:
    State();
    trans_node nodes[MAX_NODES];


};

#endif