#define ENCA 2
#define ENCB 4


volatile int64_t pose = 0;


int64_t readEncoder() {
    if (digitalRead(ENCB) > 0) {
        pose++;
    }
    else {
        pose--;
    }
}


void setup() {
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pinMode(A0, OUTPUT);
    analogWrite(A0, 255);

    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
    
    Serial.begin(2000000);
    Serial.setTimeout(0);
    analogWrite(5, 40);
}

void loop() {

    Serial.println((int)pose);
    
#if 0
    Serial.print(digitalRead(ENCA));
    Serial.print(" ");
    Serial.println(digitalRead(ENCB));
    delay(100);
#endif
}
