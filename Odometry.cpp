#include "Odometry.h"
#include "types.h"

Odometry::Odometry(){

}

Odometry::Odometry(Robot* _robot){
    this->robot = _robot;
    fl = robot->getMotor(WHEEL::FRONT_LEFT);
    fr = robot->getMotor(WHEEL::FRONT_RIGHT);
    br = robot->getMotor(WHEEL::BACK_RIGHT);
    bl = robot->getMotor(WHEEL::BACK_LEFT);
}

void Odometry::update(){
    float av_fl = fl->getAngularVelocity();
    float av_fr = fr->getAngularVelocity();
    float av_bl = bl->getAngularVelocity();
    float av_br = br->getAngularVelocity();
    v_x = (av_fl + av_fr + av_bl + av_br) * WHEEL_RADIUS * 0.25;
    v_y = (-av_fl + av_fr + av_bl - av_br) * WHEEL_RADIUS * 0.25;   
    a_x = 0; //Update get gyro data
    a_y = 0; //Update get gyro data
    p_x += v_x*TIME_INTERVAL + 0.5*a_x*TIME_INTERVAL*TIME_INTERVAL;
    p_y += v_y*TIME_INTERVAL + 0.5*a_y*TIME_INTERVAL*TIME_INTERVAL;
}

float Odometry::getPosX(){
    return p_x;
}

float Odometry::getPosY(){
    return p_y;
}