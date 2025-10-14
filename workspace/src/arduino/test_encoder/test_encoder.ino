#define ENCA 18
#define ENCB 14


int64_t pose = 0;


void readEncoder() {
    int b = digitalRead(ENCB);
    if (b > 0) {
        pose++;
    }
    else {
        pose--;
    }
}


void setup() {
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
    
    Serial.begin(115200);
    Serial.setTimeout(0);
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
