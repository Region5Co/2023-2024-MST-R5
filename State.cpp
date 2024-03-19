#include "State.h"


State::State(){
}

State::State(trans_node transitions[]){

}

void State::addNode(trans_node node)
{
    this->nodes[sizeof(nodes)+1] = node;
}

int State::init(Robot *r)
{
    return 0;
}

int State::end(){
    return 0;
}

int State::execute(){
    return 0;
}