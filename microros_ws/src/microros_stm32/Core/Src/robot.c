#include <stdbool.h>
#include <stdint.h>

#include "gpio.h"
#include "robot.h"


Motor motor_left, motor_right;


uint64_t get_current_time_ms(void) {
    return HAL_GetTick();
}


void initMotor(Motor* motor) {
    motor->ticks = 0;
    motor->prev_ticks = 0;
    motor->target = 0;
    motor->w = 0;
    motor->e_integral = 0;
    motor->e_prev = 0;
    motor->velocity = 0;

    motor->prev_time = get_current_time_ms();
}


void initMotors() {
    set_pwm(0, 0);
    initMotor(&motor_left);
    initMotor(&motor_right);
    motor_left.left = true;
    motor_right.left = false;
}


void resetMotor(Motor* motor) {
    set_pwm(0, 0);
    initMotor(motor);
}


void resetMotors() {
    resetMotor(&motor_left);
    resetMotor(&motor_right);
}


void increment_ticks_l() {
    motor_left.ticks++;
}


void increment_ticks_r() {
    motor_right.ticks++;
}


void decrement_ticks_l() {
    motor_left.ticks--;
}


void decrement_ticks_r() {
    motor_right.ticks--;
}


int64_t get_ticks_l() {
    return motor_left.ticks;
}


int64_t get_ticks_r() {
    return motor_right.ticks;
}


void __set_pwm_with_dir_l(uint8_t dir, uint8_t pwm) {
    if (dir == 1) {
        set_pwm_lb(0);
        set_pwm_lf(pwm);
    }
    else {
        set_pwm_lb(pwm);
        set_pwm_lf(0);
    }
}


void __set_pwm_with_dir_r(uint8_t dir, uint8_t pwm) {
    if (dir == 1) {
        set_pwm_rb(0);
        set_pwm_rf(pwm);
    }
    else {
        set_pwm_rb(pwm);
        set_pwm_rf(0);
    }
}


void set_pwm_l(float pwm) {
    uint8_t dir = pwm < 0 ? 0 : 1;
    pwm = fabs(pwm);
    pwm = pwm > 255 ? 255 : pwm;
    __set_pwm_with_dir_l(dir, pwm);
}


void set_pwm_r(float pwm) {
    uint8_t dir = pwm < 0 ? 0 : 1;
    pwm = fabs(pwm);
    pwm = pwm > 255 ? 255 : pwm;
    __set_pwm_with_dir_r(dir, pwm);
}


void set_pwm(float pwm_l, float pwm_r) {
    set_pwm_l(pwm_l);
    set_pwm_r(pwm_r);
}


float get_vel_l() {
    return motor_left.velocity;
}


float get_vel_r() {
    return motor_right.velocity;
}


void cmd_vel_callback(float wl, float wr) {
    motor_left.w = wl;
    motor_right.w = wr;
}


float pid(float dt, float kp, float ki, float kd, Motor* motor) {    
    float e = motor->ticks - motor->target;

    float P = e;
    motor->e_integral += e * dt;
    float I = motor->e_integral;
    float D = (e - motor->e_prev) / dt;

    motor->e_prev = e;

    float u = (kp * P + ki * I + kd * D);
    
    return u;
}


void spin(Motor* motor) {
    size_t curr_time = get_current_time_ms();
    float dt = (curr_time - motor->prev_time) / 1000.;
    
    motor->velocity = 6.28 * (motor->ticks - motor->prev_ticks) / dt / TPR;

    float n = motor->w / 6.28;

    if (n == 0.0) {
        motor->target = motor->prev_ticks;
        if (motor->left) {
            __set_pwm_with_dir_l(0, 0);
        }
        else {
            __set_pwm_with_dir_r(0, 0);
        }
    }
    else {
        float position_change = n * dt * TPR;
        motor->target += position_change;
        float u = pid(dt, KP, KI, KD, motor);

        int dir = u < 0 ? 1 : 0;
    
        uint8_t pwm = (uint8_t)fabs(u);
        pwm = pwm > 255 ? 255 : pwm;
    
        if (motor->left) {
            __set_pwm_with_dir_l(dir, pwm);
        }
        else {
            __set_pwm_with_dir_r(dir, pwm);
        }
    }
    
    motor->prev_time = curr_time;
    motor->prev_ticks = motor->ticks;
}


void spinMotors() {
    spin(&motor_left);
    spin(&motor_right);
}
