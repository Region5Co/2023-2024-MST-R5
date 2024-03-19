#ifndef IEEE_FADE_STATE_H
#define IEEE_FADE_STATE_H

#include "../Robot.h"
#include "../State.h"


class FadeState: public State{
    int init(Robot* r);
    int execute();
    int end();
    int i=0,
        j=1;
};

#endif