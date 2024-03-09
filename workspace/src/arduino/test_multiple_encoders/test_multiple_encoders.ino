#define MOTOR_0_ENCB 14 // GREEN
#define MOTOR_1_ENCB 15
#define MOTOR_2_ENCB 16
#define MOTOR_3_ENCB 17

#define MOTOR_0_ENCA 18 // YELLOW
#define MOTOR_1_ENCA 19
#define MOTOR_2_ENCA 20
#define MOTOR_3_ENCA 21


int64_t pose0 = 0;
int64_t pose1 = 0;
int64_t pose2 = 0;
int64_t pose3 = 0;


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


void setup() {
    pinMode(MOTOR_0_ENCA, INPUT);
    pinMode(MOTOR_0_ENCB, INPUT);

    pinMode(MOTOR_1_ENCA, INPUT);
    pinMode(MOTOR_1_ENCB, INPUT);

    pinMode(MOTOR_2_ENCA, INPUT);
    pinMode(MOTOR_2_ENCB, INPUT);

    pinMode(MOTOR_3_ENCA, INPUT);
    pinMode(MOTOR_3_ENCB, INPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCA), [] () {readEncoder(MOTOR_2_ENCB, &pose2);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCA), [] () {readEncoder(MOTOR_3_ENCB, &pose3);}, RISING);

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
