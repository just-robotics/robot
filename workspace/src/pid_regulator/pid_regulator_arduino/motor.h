#ifndef PID_REGULATOR_MOTOR_H
#define PID_REGULATOR_MOTOR_H


#include "pid.h"

const uint8_t MOTORS = 4;
int64_t poses[MOTORS];


class Motor {
private:
    uint8_t enca_;
    uint8_t encb_;
    uint8_t pwm_pin_;
    uint8_t f_pin_;
    uint8_t b_pin_;

    volatile int64_t pose_;
    int64_t target_;

    int8_t encoder_inc_;

    Pid pid_;

public:
    Motor(uint8_t enca, uint8_t encb, uint8_t pwm_pin, uint8_t f_pin, uint8_t b_pin);
    ~Motor() = default;

private:
    static void readEncoder(const uint8_t encb, int64_t* pose, const int8_t encoder_inc);

public:
    static void init();

    void set_target(int64_t target);
    void reset_pose();

private:
    void setPwm(uint8_t dir, uint8_t pwm);
    void spin();

public:
    static void set_poses();
    static void spinMotors();

    int64_t pose();
};


Motor motor0(MOTOR_0_ENCA, MOTOR_0_ENCB, MOTOR_0_PWM_PIN, MOTOR_0_F_PIN, MOTOR_0_B_PIN);
Motor motor1(MOTOR_1_ENCA, MOTOR_1_ENCB, MOTOR_1_PWM_PIN, MOTOR_1_F_PIN, MOTOR_1_B_PIN);
Motor motor2(MOTOR_2_ENCA, MOTOR_2_ENCB, MOTOR_2_PWM_PIN, MOTOR_2_F_PIN, MOTOR_2_B_PIN);
Motor motor3(MOTOR_3_ENCA, MOTOR_3_ENCB, MOTOR_3_PWM_PIN, MOTOR_3_F_PIN, MOTOR_3_B_PIN);


Motor::Motor(uint8_t enca, uint8_t encb, uint8_t pwm_pin, uint8_t f_pin, uint8_t b_pin) {
    enca_ = enca;
    encb_ = encb;
    pwm_pin_ = pwm_pin_;
    f_pin_ = f_pin;
    b_pin = b_pin_;

    pinMode(enca_, INPUT);
    pinMode(encb_, INPUT);
    
    pinMode(pwm_pin_, OUTPUT);
    pinMode(f_pin_, OUTPUT);
    pinMode(b_pin_, OUTPUT);

    pose_ = 0;
    target_ = 0;

    encoder_inc_ = (encb_ == MOTOR_0_ENCB || encb_ == MOTOR_2_ENCB) ? 1 : -1;

    pid_ = Pid();
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


void Motor::set_target(int64_t target) {
    target_ = target;
}


void Motor::reset_pose() {
    pose_ = 0;
    pid_.reset();
}


void Motor::setPwm(uint8_t dir, uint8_t pwm) {
    if (dir == 1) {
        digitalWrite(b_pin_, LOW);
        digitalWrite(f_pin_, HIGH);
    }
    else {
        digitalWrite(f_pin_, LOW);
        digitalWrite(b_pin_, HIGH);
    }
    analogWrite(pwm_pin_, pwm);
}


void Motor::spin() {
    float u = pid_.pid(pose_, target_);
    
    uint8_t pwm = (uint8_t)fabs(u);
    pwm = pwm > 255 ? 255 : pwm;

    int dir = u > 0 ? 1 : 0;
/*
    if (enca_ == MOTOR_0_ENCA) {
        Serial.print((int)pose_);
        Serial.print(" ");
        Serial.print((int)target_);
        Serial.print(" ");
        Serial.println(u);
        Serial.println("");
    }
*/
    setPwm(dir, pwm);
}


void Motor::set_poses() {
    poses[0] = motor0.pose_;
    poses[1] = motor1.pose_;
    poses[2] = motor2.pose_;
    poses[3] = motor3.pose_;
}


void Motor::spinMotors() {
    motor0.spin();
    motor1.spin();
    motor2.spin();
    motor3.spin();
    set_poses();
}


int64_t Motor::pose() {
    return pose_;
}


#endif // PID_REGULATOR_MOTOR_H
