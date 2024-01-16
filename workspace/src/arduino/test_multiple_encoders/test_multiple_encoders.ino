#define MOTOR_0_ENCA 18
#define MOTOR_1_ENCA 19
#define MOTOR_2_ENCA 20
#define MOTOR_3_ENCA 21

#define MOTOR_0_ENCB 14
#define MOTOR_1_ENCB 15
#define MOTOR_2_ENCB 16
#define MOTOR_3_ENCB 17


int64_t pose0 = 0;
int64_t pose1 = 0;
int64_t pose2 = 0;
int64_t pose3 = 0;


int64_t readEncoder0(int encb) {
    int b = digitalRead(encb);
    if (b > 0) {
        pose0++;
    }
    else {
        pose0--;
    }
}


int64_t readEncoder1(int encb) {
    int b = digitalRead(encb);
    if (b > 0) {
        pose1--;
    }
    else {
        pose1++;
    }
}


int64_t readEncoder2(int encb) {
    int b = digitalRead(encb);
    if (b > 0) {
        pose2++;
    }
    else {
        pose2--;
    }
}


int64_t readEncoder3(int encb) {
    int b = digitalRead(encb);
    if (b > 0) {
        pose3--;
    }
    else {
        pose3++;
    }
}


void setup() {
    pinMode(MOTOR_0_ENCA, INPUT);
    pinMode(MOTOR_0_ENCB, INPUT);

    pinMode(MOTOR_1_ENCA, INPUT);
    pinMode(MOTOR_1_ENCB, INPUT);

    pinMode(MOTOR_2_ENCA, INPUT);
    pinMode(MOTOR_2_ENCB, INPUT);

    pinMode(MOTOR_3_ENCA, INPUT);
    pinMode(MOTOR_3_ENCB, INPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder0(MOTOR_0_ENCB);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder1(MOTOR_1_ENCB);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCA), [] () {readEncoder2(MOTOR_2_ENCB);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCA), [] () {readEncoder3(MOTOR_3_ENCB);}, RISING);

    Serial.begin(2000000);
    Serial.setTimeout(0);
}

void loop() {
    Serial.print((int)pose0);
    Serial.print(" ");
    Serial.print((int)pose1);
    Serial.print(" ");
    Serial.print((int)pose2);
    Serial.print(" ");
    Serial.println((int)pose3);
}
