#ifndef INIT_STATE_H
#define INIT_STATE_H

#include "../Robot.h"
#include "../State.hpp"

class InitState: public State{
public:
    InitState();
    InitState(State::trans_node[], int);
    int init(Robot*);

    int execute();
    int end();
};

/*
* @brief Initializer takes no parameters
*/
InitState::InitState(){ name = "Init";};

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
InitState::InitState(State::trans_node _nodes[],int s){
    name="Init";
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
int InitState::init(Robot* r){
    this->robot = r;
    return 0;
}

/* 
* @brief ---------------------------
*/
int InitState::execute(){
    //Initilize
  

    robot->init();

    return 0;
}

/* 
* @brief Close any loose end and before terminating state
*/
int InitState::end(){
    robot->stop();
    return 0;
}



#endif