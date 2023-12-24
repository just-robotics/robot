#ifndef pid_h
#define pid_h


#include <stdint.h>


#define ENCA           2 // YELLOW
#define ENCB           3 // GREEN

#define F_PIN         10
#define B_PIN         11

#define KP_IDX         2
#define KD_IDX         6
#define KI_IDX        10

#define KP_SIZE        4
#define KD_SIZE        4
#define KI_SIZE        4

#define PID_CMD_SIZE  15

#define POSE_IDX       2
#define TARGET_IDX    11
#define U_IDX         20

#define POSE_SIZE      9
#define TARGET_SIZE    9
#define U_SIZE         9

#define POSE_MSG_SIZE 30


uint64_t prev_time = 0;
int64_t pose = 0;
float e_prev = 0;
float e_integral = 0;
float u = 0;

int64_t target = 200;

float kp = 1.0;
float kd = 0.0;
float ki = 0.0;

uint8_t pose_msg[POSE_MSG_SIZE];


int pin = LOW;


void setMotor(int dir, int pwm) {
    if (dir == 1) {
        analogWrite(B_PIN, LOW);
        analogWrite(F_PIN, pwm);
    }
    else if (dir == 0) {
        analogWrite(F_PIN, LOW);
        analogWrite(B_PIN, pwm);
    }
}


void pid() {
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
/*
    Serial.print((int)pose);
    Serial.print(" ");
    Serial.print((int)target);
    
    Serial.print(" ");
    Serial.print((int)e);
    Serial.print(" ");
    Serial.println((int)u);*/

    float pwm = fabs(u);
    pwm = pwm > 255 ? 255 : pwm;

    int dir = u > 0 ? 1 : 0;

    setMotor(dir, pwm);
}


void int64_to_uint8arr(int64_t number, uint8_t* output) {
    uint8_t byte = 0x000000FF;

    output[8] = number < 0 ? 0 : 1;
    uint64_t u_number = abs(number);

    output[0] = u_number & byte;
    
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        output[i] = u_number & byte;
    }
}


void setPidVariables(uint8_t* data) {
    memcpy(&kp, data[KP_IDX], KP_SIZE);
    memcpy(&kd, data[KD_IDX], KD_SIZE);
    memcpy(&ki, data[KI_IDX], KI_SIZE);
}


void setPoseVariables(uint8_t* data) {
    int64_to_uint8arr(pose, data + POSE_IDX);
    int64_to_uint8arr(target, data + TARGET_IDX);
    int64_to_uint8arr((int64_t)u, data + U_IDX);
}


#endif // pid_h
