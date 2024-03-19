#ifndef STATE_H
#define STATE_H

#define MAX_NODES 3

#include "Robot.h"
#include "Triggers.h"

typedef bool (*Trigger)(Robot*); //in the parenthesis add any parameters to pass in all transitions

class State{
friend class StateMachine;
public:
    typedef struct trans_node{
        State* next_state;       // pointer to state to goto
        Trigger* trigger;        //function pointer
    } trans_node;

    State(trans_node transitions[]);
    void addNode(trans_node node);
    int init(Robot* r);
    int execute();
    int end();
    State();
private:
    

    trans_node nodes[MAX_NODES];


};

#endif