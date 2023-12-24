#include "connection.h"
#include "pid.h"


#define BUTTON_PIN 7


//callback pid_callback = [] (uint8_t* data) {setPidVariables(data);};
callback pid_callback = [] (uint8_t* data) {setPidVariables(data);};


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
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
  
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);
  
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

    Connection::setConnection();
}


void loop() {
    //Connection::receiveCommand(PID_CMD_SIZE, pid_callback);
    setPoseVariables(pose_msg);
    Connection::sendMessage(pose_msg, POSE_MSG_SIZE);
    pid();
    delay(100);


    if (digitalRead(BUTTON_PIN) == HIGH) {
        pose = 0;
        u = 0;
        prev_time = 0;
        e_prev = 0;
        e_integral = 0;
        delay(1000);
    }
    
    //Serial.println((int)pose);
}
