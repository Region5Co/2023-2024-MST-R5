#include "Triggers.h"



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

const bool b0(Robot* r){    
    bool x =  analogRead(IEEE_B0);
    
    #ifdef IEEE_SERIAL
        Serial.print("6X: ");
        Serial.println(x);
    #endif

    //threshold needed because of noise as opposed to digital read
    return x>715;
}
const bool b1(Robot *r){
    bool x =  analogRead(IEEE_B1);
    
    #ifdef IEEE_SERIAL
        Serial.print("7X: ");
        Serial.println(x);
    #endif

    //threshold needed because of noise as opposed to digital read
    return x>715;
}


const bool b2(Robot* r){
    return false;
}