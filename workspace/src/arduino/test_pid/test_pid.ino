#define MOTOR_0_F_PIN 2
#define MOTOR_0_B_PIN 3

#define MOTOR_1_F_PIN 4
#define MOTOR_1_B_PIN 5

#define MOTOR_2_F_PIN 6
#define MOTOR_2_B_PIN 7

#define MOTOR_3_F_PIN 8
#define MOTOR_3_B_PIN 9

#define MOTOR_0_ENCB 14 // GREEN
#define MOTOR_1_ENCB 15
#define MOTOR_2_ENCB 16
#define MOTOR_3_ENCB 17

#define MOTOR_0_ENCA 18 // YELLOW
#define MOTOR_1_ENCA 19
#define MOTOR_2_ENCA 20
#define MOTOR_3_ENCA 21


typedef struct Motor {
    int dir;
    float pwm;
} Motor;


uint64_t prev_time = 0;
    
float e_prev = 0;
float e_integral = 0;
    
volatile int64_t pose0 = 0;
volatile int64_t pose1 = 0;
volatile int64_t pose2 = 0;
volatile int64_t pose3 = 0;

int64_t target = 200;

float kp = 1.0;
float kd = 0.0;
float ki = 0.0;


void reset() {
    pose0 = 0;
    pose1 = 0;
    pose2 = 0;
    pose3 = 0;
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


void setMotor(Motor m, int f_pin, int b_pin) {
    if (m.dir == 1) {
        digitalWrite(b_pin, LOW);
        digitalWrite(f_pin, m.pwm);
    }
    else {
        digitalWrite(f_pin, LOW);
        digitalWrite(b_pin, m.pwm);
    }
}


Motor pid(int64_t pose, int64_t target) {    
    uint64_t current_time = micros();

    float dt = (float)(current_time - prev_time) / 1.0e6;
    prev_time = current_time;

    int64_t e = pose - target;

    float P = float(e);
    float D = (e - e_prev) / dt;
    e_integral += e * dt;
    float I = e_integral;

    float u = (kp * P + kd * D + ki * I);

    e_prev = e;

    Motor motor;

    motor.pwm = fabs(u);
    motor.pwm = motor.pwm > 255 ? 255 : motor.pwm;

    motor.dir = u > 0 ? 0 : 1;

    return motor;
}


void setup() {   
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

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCA), [] () {readEncoder(MOTOR_2_ENCB, &pose2);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCA), [] () {readEncoder(MOTOR_3_ENCB, &pose3);}, RISING);

    Serial.begin(2000000);
    Serial.setTimeout(0);

    prev_time = 0;
}

void loop() {
    Motor u0 = pid(pose1, target);
    Motor u1 = pid(pose1, target);
    Motor u2 = pid(pose2, target);
    Motor u3 = pid(pose3, target);

    setMotor(u0, MOTOR_0_F_PIN, MOTOR_0_B_PIN);
    setMotor(u1, MOTOR_1_F_PIN, MOTOR_1_B_PIN);
    setMotor(u2, MOTOR_2_F_PIN, MOTOR_2_B_PIN);
    setMotor(u3, MOTOR_3_F_PIN, MOTOR_3_B_PIN);
}
