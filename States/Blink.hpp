#ifndef IEEE_BLINK_STATE_H
#define IEEE_BLINK_STATE_H

#include "../Robot.h"
#include "../State.hpp"


class BlinkState: public State{
public:
    
    BlinkState();
    BlinkState(State::trans_node _nodes[],int s);
    int init(Robot*);

    int execute();
    int end();

    int i,j;
};

/*
* @brief Initializer takes no parameters
*/
BlinkState::BlinkState():i(0), j(0){ name = "Blink"; };

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
BlinkState::BlinkState(State::trans_node _nodes[],int s){
    name="Blink";
    this->size = s;
    this->i = 0;
    this->j = 0;

    //Adding nodes to class
    for(int i=0; i<=s;i++){
   		State::trans_node* n = &_nodes[i];
   		this->addNode(n,s-i);
    }
}

/*
* @brief Initialize FadeState
* @param robot pointer to robot object
*/
int BlinkState::init(Robot* r){
    return 0;
}

/* 
* @brief This state execution is meant to turn on an LED
*/
int BlinkState::execute(){
    digitalWrite(LED_BUILTIN, i);
    
    j++;
    if (j >= 2){
        if(i==0){i=1;}
        else {i=0;}
        j=0;
    }
    delay(10);
    return 0;
}

/* 
* @brief Close any loose end and before terminating state
*/
int BlinkState::end(){
    return 0;
}

#endif