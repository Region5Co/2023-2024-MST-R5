#include "Triggers.h"
#define IEEE_B0 7
#define IEEE_B1 8


bool pinHigh(Robot *r)
{
    return false;
}

bool closeDistance(Robot *robot)
{
    return 6 > 5;
}

bool readSomething(Robot* robot)
{
    return 0;
}

bool b0(Robot* r){
    delay(10);
    return digitalRead(IEEE_B0);
}

bool b1(Robot *r){
    delay(10);
    return digitalRead(IEEE_B1);
}
