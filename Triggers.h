#ifndef TRIGGERS_H
#define TRIGGERS_H

#include "Robot.h"
#include "State.hpp"
#include "IEEE_Pinout.h"


//Add headers for functions for transitions  **Ignore**
bool pinHigh(Robot* r);

bool closeDistance(Robot* r);

bool readSomething(Robot* r);

//Test triggers
const bool b0(Robot*);
const bool b1(Robot*);
const bool b2(Robot*);


int scanTriggers(State* state);


#endif