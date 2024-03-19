#ifndef IEEE_BLINK_STATE_H
#define IEEE_BLINK_STATE_H

#include "../Robot.h"
#include "../State.h"


class BlinkState: public State{
    int init(Robot* r);
    int execute();
    int end();
    int i=0,
        j=0;
};

#endif