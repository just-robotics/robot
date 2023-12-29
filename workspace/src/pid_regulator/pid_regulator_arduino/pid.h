#ifndef PID_REGULATOR_PID_H
#define PID_REGULATOR_PID_H


#include <stdint.h>

#include "blink.h"
#include "connection.h"
#include "config.h"


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


void Pid::readEncoder() {
    int b = digitalRead(ENCB);
    if (b > 0) {
        pose++;
    }
    else {
        pose--;
    }
}


void Pid::setMotor(int dir, int pwm) {
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


void Pid::setPoseVariables(uint8_t* data) {
    Connection::int64_to_uint8arr((int64_t)kp, data + POSE_IDX);
    Connection::int64_to_uint8arr((int64_t)kd, data + TARGET_IDX);
    Connection::int64_to_uint8arr((int64_t)ki, data + U_IDX);
}

/*
void Pid::setPoseVariables(uint8_t* data) {    
    Connection::int64_to_uint8arr(pose, data + POSE_IDX);
    Connection::int64_to_uint8arr(target, data + TARGET_IDX);
    Connection::int64_to_uint8arr((int64_t)u, data + U_IDX);
}
*/

#endif // PID_REGULATOR_PID_H
