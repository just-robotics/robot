#define MOTOR_0_F_PIN 5
#define MOTOR_0_B_PIN 6

#define MOTOR_1_F_PIN 9
#define MOTOR_1_B_PIN 10

#define MOTOR_0_ENCB 4 // GREEN
#define MOTOR_1_ENCB 7

#define MOTOR_0_ENCA 2 // YELLOW
#define MOTOR_1_ENCA 3


typedef struct Motor {
    int dir;
    float pwm;
} Motor;


uint64_t prev_time = 0;
    
float e_prev = 0;
float e_integral = 0;
    
volatile int64_t pose0 = 0;
volatile int64_t pose1 = 0;

int64_t target = 200;

float kp = 1.0;
float kd = 0.0;
float ki = 0.0;


void reset() {
    pose0 = 0;
    pose1 = 0;
    prev_time = 0;
    e_prev = 0;
    e_integral = 0;
    delay(200);
}


void readEncoder(int encb, int64_t* pose) {
    int k = (encb == MOTOR_0_ENCB) ? 1 : -1;
    if (digitalRead(encb) > 0) {
        *pose = *pose + k;
    }
    else {
        *pose = *pose - k;
    }
}


void setMotor(Motor m, int f_pin, int b_pin) {
    if (m.dir == 1) {
        analogWrite(b_pin, LOW);
        analogWrite(f_pin, m.pwm);
    }
    else {
        analogWrite(f_pin, LOW);
        analogWrite(b_pin, m.pwm);
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

    pinMode(MOTOR_0_ENCB, INPUT);
    pinMode(MOTOR_1_ENCB, INPUT);

    pinMode(MOTOR_0_F_PIN, OUTPUT);
    pinMode(MOTOR_0_B_PIN, OUTPUT);
    pinMode(MOTOR_1_F_PIN, OUTPUT);
    pinMode(MOTOR_1_B_PIN, OUTPUT);

    pinMode(A0, OUTPUT);
    analogWrite(A0, 255);

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);

    Serial.begin(2000000);
    Serial.setTimeout(0);

    prev_time = 0;
}

void loop() {
    Motor u0 = pid(pose0, target);
    Motor u1 = pid(pose1, target);

    setMotor(u0, MOTOR_0_F_PIN, MOTOR_0_B_PIN);
    setMotor(u1, MOTOR_1_F_PIN, MOTOR_1_B_PIN);
}
