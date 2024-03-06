#ifndef STATE_H
#define STATE_H

#define MAX_NODES 3

typedef bool (*Transistion)(); //in the parenthesis add any parameters to pass in all transitions

class State{
friend class StateMachine;
public:
    typedef struct trans_node{
        State* state;       // pointer to state to goto
        Transistion* trans; //function pointer
    } trans_node;

    State(trans_node transitions[]);
    int init();
    int execute();
    int end();
private:
    State();
    trans_node nodes[MAX_NODES];


};






#endif