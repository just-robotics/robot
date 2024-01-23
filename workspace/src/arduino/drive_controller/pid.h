#ifndef DRIVE_CONTROLLER_PID_H
#define DRIVE_CONTROLLER_PID_H


#include "config.h"


class Pid {
private:
    float e_prev_;
    float e_integral_;

public:
    float pid(int64_t pose, int64_t target, float dt);
};


float Pid::pid(int64_t pose, int64_t target, float dt) {    
    float e = float(pose - target);

    float P = e;
    e_integral_ += e * dt;
    float I = e_integral_;
    float D = (e - e_prev_) / dt;

    e_prev_ = e;

    float u = (KP * P + KI * I + KD * D);
    
    return u;
}


#endif // DRIVE_CONTROLLER_PID_H
