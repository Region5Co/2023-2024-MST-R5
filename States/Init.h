#ifndef IEEE_INIT_STATE_H
#define IEEE_INIT_STATE_H

#include "../Robot.h"
#include "../State.h"


class InitState: public State{
    int init(Robot* r);
    int execute();
    int end();
};

#endif