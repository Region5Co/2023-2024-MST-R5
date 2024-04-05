#ifndef IEEE_SOLIDSTATE_H
#define IEEE_SOLIDSTATE_H

#include "../Robot.h"
#include "../State.hpp"


class SolidState: public State{
public:

    SolidState();
    SolidState(State::trans_node _nodes[], int s);
    int init(Robot*);

    int execute();
    int end();
    
    int i,j;
};

/*
* @brief Initializer takes no parameters
*/
SolidState::SolidState() : i(0), j(0) { name = "Solid";};

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
SolidState::SolidState(State::trans_node _nodes[], int s){
    name="Solid";
    this->size = s;
    this->i = 0;
    this->j = 0;

    //Adding nodes to class
    for(int i=0; i<=s;i++){
   		State::trans_node* n = &_nodes[i];
   		this->addNode(n,s-i);
    }
};

/*
* @brief Initialize FadeState
* @param robot pointer to robot object
*/
int SolidState::init(Robot* robot){
    return 0;
}

/* 
* @brief This state execution is meant to turn on an LED
*/
int SolidState::execute(){
    Serial.println("Solid");
    delay(10);
    digitalWrite(LED_BUILTIN,HIGH);
    return 0;
}

/* 
* @brief Close any loose end and before terminating state
*/
int SolidState::end(){
    return 0;
}

#endif