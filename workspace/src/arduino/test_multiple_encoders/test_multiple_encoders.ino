#define MOTOR_0_ENCB 4 // GREEN
#define MOTOR_1_ENCB 7

#define MOTOR_0_ENCA 2 // YELLOW
#define MOTOR_1_ENCA 3


int64_t pose0 = 0;
int64_t pose1 = 0;


void readEncoder(int encb, int64_t* pose) {
    int k = (encb == MOTOR_0_ENCB) ? 1 : -1;
    if (digitalRead(encb) > 0) {
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

    pinMode(A0, OUTPUT);
    analogWrite(A0, 255);

    attachInterrupt(digitalPinToInterrupt(MOTOR_0_ENCA), [] () {readEncoder(MOTOR_0_ENCB, &pose0);}, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA), [] () {readEncoder(MOTOR_1_ENCB, &pose1);}, RISING);

    Serial.begin(2000000);
    Serial.setTimeout(0);
}

void loop() {
    Serial.print((int)pose0);
    Serial.print(" ");
    Serial.print((int)pose1);
    Serial.println();
}
