#define MOTOR_0_F_PIN 10
#define MOTOR_0_B_PIN 9

#define MOTOR_1_F_PIN 6
#define MOTOR_1_B_PIN 5

#define MOTOR_0_ENCB 4 // GREEN
#define MOTOR_1_ENCB 7

#define MOTOR_0_ENCA 2 // YELLOW
#define MOTOR_1_ENCA 3


typedef struct Motor {
    int dir;
    float pwm;
} Motor;


volatile int64_t pose0 = 0;
volatile int64_t pose1 = 0;


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


void setup() {    
    pinMode(MOTOR_0_ENCA, INPUT);
    pinMode(MOTOR_1_ENCA, INPUT);

    pinMode(A0, OUTPUT);
    analogWrite(A0, 255);

    pinMode(MOTOR_0_ENCB, INPUT);
    pinMode(MOTOR_1_ENCB, INPUT);

    pinMode(MOTOR_0_F_PIN, OUTPUT);
    pinMode(MOTOR_0_B_PIN, OUTPUT);
    pinMode(MOTOR_1_F_PIN, OUTPUT);
    pinMode(MOTOR_1_B_PIN, OUTPUT);
  
    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);

    Serial.begin(2000000);
    Serial.setTimeout(0);
}


void loop() {

    Motor m;
    m.dir = 1;
    m.pwm = 0;

    setMotor(m, MOTOR_0_F_PIN, MOTOR_0_B_PIN);
    setMotor(m, MOTOR_1_F_PIN, MOTOR_1_B_PIN);

    Serial.print((int)pose0);
    Serial.print(" ");
    Serial.print((int)pose1);
    Serial.println();
}
