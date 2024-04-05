#ifndef IEEE_FADESTATE_H
#define IEEE_FADESTATE_H

#include "../Robot.h"
#include "../State.hpp"


class FadeState : public State{
public:
    
    FadeState();
    FadeState(State::trans_node _nodes[],int s);
    int init(Robot*);

    int execute() ;
    int end() ;
       
    int i,j;
};

/*
* @brief Initializer takes no parameters
*/
FadeState::FadeState() : i(200), j(0) {};

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
FadeState::FadeState(State::trans_node _nodes[], int s) 
{
    this->size = s;
    for(int i=0; i<=s;i++){
		State::trans_node* n = &_nodes[i];
        this->addNode(n,s-i);
		
    }
    i=200;
    name="Fade";
    j=0;
};

/*
* @brief Initialize FadeState
* @param robot pointer to robot object
*/
int FadeState::init(Robot* robot){
    return 0;
}

/* 
* @brief This state execution is meant to fade an LED
*/
int FadeState::execute(){
    if(i>=255){
        j=-1;
    }else if(i<=0){
        j=1;
    }
    i+=j;
    Serial.println("Fade");
    delay(10);
    analogWrite(LED_BUILTIN, i);
    delay(1);
    return 0;
}

/* 
* @brief Close any loose end and before terminating state
*/
int FadeState::end(){
    return 0;
}

/*
* @brief Add nodes to State object array 
* @param node node to be added
* @param loc location for the node to be added in nodes array
* @warning loc should not exceed MAX_NODES
*/
void State::addNode(trans_node* node, int loc){
    this->nodes[loc] = node;
}

#endif