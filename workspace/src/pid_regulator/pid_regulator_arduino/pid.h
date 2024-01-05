#ifndef PID_REGULATOR_PID_H
#define PID_REGULATOR_PID_H


#include "config.h"


class Pid {
private:
    float kp_;
    float ki_;
    float kd_;

    uint64_t prev_time_;
    
    float e_prev_;
    float e_integral_;

public:
    Pid(float kp=KP, float ki=KI, float kd=KD);

    void reset();
    void set_target(int64_t target);
    float pid(int64_t pose, int64_t target);
};


Pid::Pid(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    reset();
}


void Pid::reset() {
    prev_time_ = 0;
    e_prev_ = 0;
    e_integral_ = 0;
}


float Pid::pid(int64_t pose, int64_t target) {    
    uint64_t current_time = micros();

    float dt = (float)(current_time - prev_time_) / 1.0e6;
    prev_time_ = current_time;

    int64_t e = pose - target;

    float P = float(e);
    e_integral_ += e * dt;
    float I = e_integral_;
    float D = (e - e_prev_) / dt;

    e_prev_ = e;

    float u = (kp_ * P + ki_ * I + kd_ * D);

    return u;
}


/*
namespace Pid {
    uint64_t prev_time = 0;
    
    float e_prev = 0;
    float e_integral = 0;
    
    int64_t pose = 0;
    int64_t target = 200;
    float u = 0;

    float kp = 1.0;
    float kd = 0.0;
    float ki = 0.0;

    void reset();
    void readEncoder();
    void setMotor(int dir, int pwm);
    void pid();
    void setPidVariables(uint8_t* data);
    void setPoseVariables(uint8_t* data);
}


void Pid::reset() {
    pose = 0;
    u = 0;
    prev_time = 0;
    e_prev = 0;
    e_integral = 0;
}


void readEncoder(int encb, int64_t* pose) {
    int b = digitalRead(encb);
    int k = (encb == MOTOR_0_ENCB || encb == MOTOR_2_ENCB) ? 1 : -1;
    if (b > 0) {
        *pose = *pose + k;
    }
    else {
        *pose = *pose - k;
    }
}


void Pid::setMotor(Motor m) {
    analogWrite(PWM_PIN, pwm);
    if (dir == 1) {
        digitalWrite(B_PIN, LOW);
        digitalWrite(F_PIN, HIGH);
    }
    else {
        digitalWrite(F_PIN, LOW);
        digitalWrite(B_PIN, HIGH);
    }
}


void Pid::pid() {    
    uint64_t current_time = micros();

    float dt = (float)(current_time - prev_time) / 1.0e6;
    prev_time = current_time;

    int64_t e = pose - target;

    float P = float(e);
    float D = (e - e_prev) / dt;
    e_integral += e * dt;
    float I = e_integral;

    u = (kp * P + kd * D + ki * I);

    e_prev = e;

    float pwm = fabs(u);
    pwm = pwm > 255 ? 255 : pwm;

    int dir = u > 0 ? 1 : 0;

    setMotor(dir, pwm);
}


void Pid::setPidVariables(uint8_t* data) {
    kp = Connection::uint8arr_to_float(data + KP_IDX);
    kd = Connection::uint8arr_to_float(data + KD_IDX);
    ki = Connection::uint8arr_to_float(data + KI_IDX);

    Blink::blink(true);
}
*/
/*
void Pid::setPoseVariables(uint8_t* data) {
    Connection::int64_to_uint8arr((int64_t)kp, data + POSE_IDX);
    Connection::int64_to_uint8arr((int64_t)kd, data + TARGET_IDX);
    Connection::int64_to_uint8arr((int64_t)ki, data + U_IDX);
}
*/
/*
void Pid::setPoseVariables(uint8_t* data) {    
    Connection::int64_to_uint8arr(pose, data + POSE_IDX);
    Connection::int64_to_uint8arr(target, data + TARGET_IDX);
    Connection::int64_to_uint8arr((int64_t)u, data + U_IDX);
}
*/

#endif // PID_REGULATOR_PID_H
