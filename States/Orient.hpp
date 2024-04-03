#ifndef ORIENT_STATE_H
#define ORIENT_STATE_H

#include "../Robot.h"
#include "../State.hpp"

class OrientState: public State{
public:
    OrientState();
    OrientState(State::trans_node[], int);
    int init(Robot*);

    int execute();
    int end();

    float targetAngle;
};

/*
* @brief Initializer takes no parameters
*/
OrientState::OrientState(){ name = "Orient";};

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
OrientState::OrientState(State::trans_node _nodes[],int s){
    name="Orient";
    this->size = s;

    //Adding nodes to class
    for(int i=0; i<=s;i++){
   		State::trans_node* n = &_nodes[i];
   		this->addNode(n,s-i);
    }
}

/*
* @brief Initialize Init State
* @param robot pointer to robot object
*/
int OrientState::init(Robot* r){
    this->robot = r;
    this->targetAngle = 10;
    return 0;
}

/* 
* @brief ---------------------------
*/
int OrientState::execute(){
    //Initilize
    

    robot->init();

    return 0;
}

/* 
* @brief Close any loose end and before terminating state
*/
int OrientState::end(){
    robot->stop();
    return 0;
}



#endif