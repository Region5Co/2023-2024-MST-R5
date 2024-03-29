#ifndef IEEE_TraversingState_H
#define IEEE_TraversingState_H

#include "../Robot.h"
#include "../State.hpp"

#define LOCALE_RADIUS 3.0
#define MOVE_BY_POSITION false
#define MOVE_BY_ANGLE true
class TraversingState: public State{
public:

    TraversingState();
    TraversingState(State::trans_node _nodes[], int s);
    int init(Robot*);

    int execute();
    int end();

};

/*
* @brief Initializer takes no parameters
*/
TraversingState::TraversingState() { name = "Traversing";};

/*
* @brief Initialize state with nodes
* @param nodes array of State::trans_node links
* @param s the size of @ref nodes[] 
*/
TraversingState::TraversingState(State::trans_node _nodes[], int s){
    name="Traversing";
    this->size = s;

    //Adding nodes to class
    for(int i=0; i<=s;i++){
   		State::trans_node* n = &_nodes[i];
   		this->addNode(n,s-i);
    }
};

/*
* @brief Initialize TraversingState
* @param robot pointer to robot object
*/
int TraversingState::init(Robot* robot){
    return 0;
}

/* 
* @brief This state execution is meant to allow the robot to move while tracking potential enemies
*/
int TraversingState::execute(){
    //If wanted to you can put the step to move away from wall here. 
    #if MOVE_BY_POSITION
        //Use Current Position and next position
        //i.e. x(n+1) - x(n) =x 
        //use x in strafe calculations
    #elif MOVE_BY_ANGLE
        //purely use gyro angles
        //i.e. offset gyro to make curr angle 0 then rotate to desired angle and go straight
        //NOTE MOVE AWAY FROM WALL BEFORE TURNING OR YOULL HIT
        //NOTE #2 IF YOU MOVE AWAY FROM WAY ANGLE WILL CHANGE SO RECALCULATE ON THE FLY
    #endif
    //If wanted you can put the part to rotate to correct orientation here
    //then move to target more than likely it would be sideways movement
}

/* 
* @brief Close any loose end and before terminating state
*/
int TraversingState::end(){
    return 0;
}

#endif