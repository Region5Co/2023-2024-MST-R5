#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Robot.h"

#define TIME_INTERVAL 1 //ns

class Odometry{
public:
    Odometry();
    Odometry(Robot*);

    void update();
    float getPosX();
    float getPosY();
private:
    Robot* robot;
    Motor* fl, *fr, *bl, *br;
    float v_x, v_y;
    float p_x, p_y;
    float a_x, a_y;
};

#endif
