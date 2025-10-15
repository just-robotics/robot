#ifndef DRIVE_CONTROLLER_MOTOR_H
#define DRIVE_CONTROLLER_MOTOR_H


#include <std_msgs/Bool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "config.h"


class Motor {
private:
    uint8_t enca_;
    uint8_t encb_;
    uint8_t f_pin_;
    uint8_t b_pin_;

    volatile int64_t pose_;
    float target_;
    float cmd_vel_;
    float velocity_;
    uint64_t prev_time_;
    int64_t prev_pose_;

    int8_t encoder_inc_;

    float e_prev_;
    float e_integral_;

    float kp, ki, kd;

public:
    Motor(uint8_t enca, uint8_t encb, uint8_t f_pin, uint8_t b_pin);
    ~Motor() = default;

private:
    static void readEncoder(const uint8_t encb, int64_t* pose, const int8_t encoder_inc);

public:
    static void init();

    void setPwm(uint8_t dir, uint8_t pwm);

    float pid(float dt, float kp, float ki, float kd);
    
    void spin();
    static void spinMotors();

    static void fillOdomMsg(std_msgs::Int64MultiArray& msg);

    void set_cmd_vel(float cmd_vel);
    
    static void setVelocities(float* vels);

    void reset();
    static void resetMotors(const std_msgs::Bool& msg);

    static void callback(const std_msgs::Float32MultiArray& msg);
};


Motor motor0(MOTOR_0_ENCA, MOTOR_0_ENCB, MOTOR_0_F_PIN, MOTOR_0_B_PIN);
Motor motor1(MOTOR_1_ENCA, MOTOR_1_ENCB, MOTOR_1_F_PIN, MOTOR_1_B_PIN);
Motor motor2(MOTOR_2_ENCA, MOTOR_2_ENCB, MOTOR_2_F_PIN, MOTOR_2_B_PIN);
Motor motor3(MOTOR_3_ENCA, MOTOR_3_ENCB, MOTOR_3_F_PIN, MOTOR_3_B_PIN);


Motor::Motor(uint8_t enca, uint8_t encb, uint8_t f_pin, uint8_t b_pin) {
    kp = KP;
    ki = KI;
    kd = KD;
    
    enca_ = enca;
    encb_ = encb;
    f_pin_ = f_pin;
    b_pin_ = b_pin;

    pinMode(enca_, INPUT);
    pinMode(encb_, INPUT);
    
    pinMode(f_pin_, OUTPUT);
    pinMode(b_pin_, OUTPUT);

    reset();

    encoder_inc_ = (encb_ == MOTOR_0_ENCB || encb_ == MOTOR_2_ENCB) ? 1 : -1;

    e_prev_ = 0;
    e_integral_ = 0;

    prev_time_  = micros();
}


void Motor::readEncoder(uint8_t encb, int64_t* pose, int8_t encoder_inc) {    
    int b = digitalRead(encb);
    if (b > 0) {
        *pose += encoder_inc;
    }
    else {
        *pose -= encoder_inc;
    }
}


void Motor::init() {
    attachInterrupt(digitalPinToInterrupt(motor0.enca_), [] () {readEncoder(motor0.encb_, &motor0.pose_, motor0.encoder_inc_);}, RISING);
    attachInterrupt(digitalPinToInterrupt(motor1.enca_), [] () {readEncoder(motor1.encb_, &motor1.pose_, motor1.encoder_inc_);}, RISING);
    attachInterrupt(digitalPinToInterrupt(motor2.enca_), [] () {readEncoder(motor2.encb_, &motor2.pose_, motor2.encoder_inc_);}, RISING);
    attachInterrupt(digitalPinToInterrupt(motor3.enca_), [] () {readEncoder(motor3.encb_, &motor3.pose_, motor3.encoder_inc_);}, RISING);
}


void Motor::setPwm(uint8_t dir, uint8_t pwm) {
    if (dir == 1) {
        analogWrite(b_pin_, LOW);
        analogWrite(f_pin_, pwm);
    }
    else {
        analogWrite(f_pin_, LOW);
        analogWrite(b_pin_, pwm);
    }
}


float Motor::pid(float dt, float kp, float ki, float kd) {    
    float e = float(pose_ - target_);

    float P = e;
    e_integral_ += e * dt;
    float I = e_integral_;
    float D = (e - e_prev_) / dt;

    e_prev_ = e;

    float u = (kp * P + ki * I + kd * D);
    
    return u;
}


void Motor::spin() {
    
    uint64_t curr_time = micros();
    float dt = (curr_time - prev_time_) / 1.0e6;

    velocity_ = 2 * 3.14 * (pose_ - prev_pose_) / dt / TPR;
    
    float n = cmd_vel_ / 2 / 3.14;

    if (n == 0.0) {
        target_ = pose_;
        setPwm(0, 0);
    }
    else {
        float position_change = n * dt * TPR;
        target_ += position_change;
        float u = pid(dt, kp, ki, kd);
    
        uint8_t pwm = (uint8_t)fabs(u);
        pwm = pwm > 255 ? 255 : pwm;

        int dir = u < 0 ? 1 : 0;
    
        setPwm(dir, pwm);
    }

    prev_time_ = micros();
    prev_pose_ = pose_;
}


void Motor::spinMotors() {
    motor0.spin();
    motor1.spin();
    motor2.spin();
    motor3.spin();
}


void Motor::fillOdomMsg(std_msgs::Int64MultiArray& msg) {
    msg.data[0] = motor0.pose_;
    msg.data[1] = motor1.pose_;
    msg.data[2] = motor2.pose_;
    msg.data[3] = motor3.pose_;
}


void Motor::set_cmd_vel(float cmd_vel) {
    cmd_vel_ = cmd_vel;
}


void Motor::reset() {
    setPwm(0, 0);
    pose_ = 0;
    target_ = 0;
    cmd_vel_ = 0;
    prev_pose_ = 0;
    e_prev_ = 0;
    e_integral_ = 0;
}


void Motor::resetMotors(const std_msgs::Bool& msg) {
    if (msg.data == false) {
        return;
    }
    
    motor0.reset();
    motor1.reset();
    motor2.reset();
    motor3.reset();
}


void Motor::callback(const std_msgs::Float32MultiArray& msg) {   
    motor0.set_cmd_vel(msg.data[0]);
    motor1.set_cmd_vel(msg.data[1]);
    motor2.set_cmd_vel(msg.data[2]);
    motor3.set_cmd_vel(msg.data[3]);
}


#endif // DRIVE_CONTROLLER_MOTOR_H
