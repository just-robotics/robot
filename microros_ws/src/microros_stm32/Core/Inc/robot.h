#ifndef MOTOR_H
#define MOTOR_H


#include <math.h>
#include <stdbool.h>


#define TPR 330.0
#define KP 2.0
#define KI 0.0
#define KD 0.1


typedef struct {
    bool left;
    volatile int64_t ticks;
    int64_t prev_ticks;
    int64_t target;
    float w;
    int64_t prev_time;
    float e_integral, e_prev;
    float velocity;
} Motor;


void increment_ticks_l();
void increment_ticks_r();

void decrement_ticks_l();
void decrement_ticks_r();

int64_t get_ticks_l();
int64_t get_ticks_r();

float get_vel_l();
float get_vel_r();

void cmd_vel_callback(float wl, float wr);

void set_pwm(float pwm_l, float pwm_r);

void initMotors();
void spinMotors();

void resetMotors();


#endif  //  MOTOR_H
