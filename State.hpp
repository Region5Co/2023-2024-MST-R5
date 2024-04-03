#ifndef STATE_H
#define STATE_H

#define MAX_NODES 3

#include "Robot.h"

typedef bool (*Trigger)(Robot*); //in the parenthesis add any parameters to pass in all transitions

class State{
friend class StateMachine;

public:

    typedef struct trans_node{
        State* next_state;       // pointer to state to goto
        Trigger trigger;        //function pointer
    } trans_node;

    virtual int init(Robot*)   =0;
    virtual int execute()      =0;
    virtual int end()          =0;
    
    //@brief Defined in state machine's initial state 
    //      due to compiling error
    void addNode(trans_node* node, int s);
    
    //Nodes used for transitions
    trans_node* nodes[MAX_NODES];
    int size;

    //Name of State - used for debugging
    String name;
protected:
    Robot* robot;
};


#endif