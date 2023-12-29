#include <stdint.h>


#define MOTOR_0_PWM_PIN 5
#define MOTOR_1_PWM_PIN 6
#define MOTOR_2_PWM_PIN 9
#define MOTOR_3_PWM_PIN 10

#define MOTOR_0_ENCA 0 // YELLOW
#define MOTOR_1_ENCA 1
#define MOTOR_2_ENCA 2
#define MOTOR_3_ENCA 3

#define MOTOR_0_ENCB 4 // GREEN
#define MOTOR_1_ENCB 7
#define MOTOR_2_ENCB 8
#define MOTOR_3_ENCB 11

#define MOTOR_0_F_PIN 12
#define MOTOR_0_B_PIN 13

#define MOTOR_1_F_PIN A0
#define MOTOR_1_B_PIN A1

#define MOTOR_2_F_PIN A2
#define MOTOR_2_B_PIN A3

#define MOTOR_3_F_PIN A4
#define MOTOR_3_B_PIN A5


typedef struct MotorU {
    int dir;
    float pwm;
} MotorU;


uint64_t prev_time = 0;
    
float e_prev = 0;
float e_integral = 0;
    
volatile int64_t pose0 = 0;
volatile int64_t pose1 = 0;
volatile int64_t pose2 = 0;
volatile int64_t pose3 = 0;

int64_t target = 200;
float u = 0;

float kp = 1.0;
float kd = 0.0;
float ki = 0.0;

uint8_t pwm = 0;


void reset() {
    pose0 = 0;
    pose1 = 0;
    pose2 = 0;
    pose3 = 0;
    u = 0;
    prev_time = 0;
    e_prev = 0;
    e_integral = 0;
    delay(200);
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


void setMotor(MotorU u, int f_pin, int b_pin, int pwm_pin) {
    analogWrite(pwm_pin, u.pwm);
    if (u.dir == 1) {
        digitalWrite(b_pin, LOW);
        digitalWrite(f_pin, HIGH);
    }
    else {
        digitalWrite(f_pin, LOW);
        digitalWrite(b_pin, HIGH);
    }
}


MotorU pid(int64_t pose, int64_t target) {    
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

    MotorU motor_u;

    motor_u.pwm = fabs(u);
    motor_u.pwm = motor_u.pwm > 255 ? 255 : motor_u.pwm;

    motor_u.dir = u > 0 ? 0 : 1;

    return motor_u;
}


void setup() {
    pinMode(MOTOR_0_PWM_PIN, OUTPUT);
    pinMode(MOTOR_1_PWM_PIN, OUTPUT);
    pinMode(MOTOR_2_PWM_PIN, OUTPUT);
    pinMode(MOTOR_3_PWM_PIN, OUTPUT);
    
    pinMode(MOTOR_0_ENCA, INPUT);
    pinMode(MOTOR_1_ENCA, INPUT);
    pinMode(MOTOR_2_ENCA, INPUT);
    pinMode(MOTOR_3_ENCA, INPUT);

    pinMode(MOTOR_0_ENCB, INPUT);
    pinMode(MOTOR_1_ENCB, INPUT);
    pinMode(MOTOR_2_ENCB, INPUT);
    pinMode(MOTOR_3_ENCB, INPUT);

    pinMode(MOTOR_0_F_PIN, OUTPUT);
    pinMode(MOTOR_0_B_PIN, OUTPUT);
    pinMode(MOTOR_1_F_PIN, OUTPUT);
    pinMode(MOTOR_1_B_PIN, OUTPUT);
    pinMode(MOTOR_2_F_PIN, OUTPUT);
    pinMode(MOTOR_2_B_PIN, OUTPUT);
    pinMode(MOTOR_3_F_PIN, OUTPUT);
    pinMode(MOTOR_3_B_PIN, OUTPUT);
  
    Serial.begin(2000000);
    Serial.setTimeout(0);

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCA), [] () {readEncoder(MOTOR_2_ENCB, &pose2);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCA), [] () {readEncoder(MOTOR_3_ENCB, &pose3);}, RISING);
}


void loop() {
    MotorU u0 = pid(pose0, target);
    MotorU u1 = pid(pose1, target);
    MotorU u2 = pid(pose2, target);
    MotorU u3 = pid(pose3, target);

    setMotor(u0, MOTOR_0_F_PIN, MOTOR_0_B_PIN, MOTOR_0_PWM_PIN);
    setMotor(u1, MOTOR_1_F_PIN, MOTOR_1_B_PIN, MOTOR_1_PWM_PIN);
    setMotor(u2, MOTOR_2_F_PIN, MOTOR_2_B_PIN, MOTOR_2_PWM_PIN);
    setMotor(u3, MOTOR_3_F_PIN, MOTOR_3_B_PIN, MOTOR_3_PWM_PIN);
/*
    Serial.print((int)pose0);
    Serial.print(" ");
    Serial.print((int)pose1);
    Serial.print(" ");
    Serial.print((int)pose2);
    Serial.print(" ");
    Serial.println((int)pose3);

*/

    //int pwm = 80;
    //setMotor(1, pwm, MOTOR_0_F_PIN, MOTOR_0_B_PIN, MOTOR_0_PWM_PIN);
    //setMotor(1, pwm, MOTOR_1_F_PIN, MOTOR_1_B_PIN, MOTOR_1_PWM_PIN);
    //setMotor(1, pwm, MOTOR_2_F_PIN, MOTOR_2_B_PIN, MOTOR_2_PWM_PIN);
    //setMotor(1, pwm, MOTOR_3_F_PIN, MOTOR_3_B_PIN, MOTOR_3_PWM_PIN);

/*
    if (Serial.available() > 0) {
        setMotor(1, SerialPerseInt());
    }
*/
}
