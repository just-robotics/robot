#include "connection.h"
#include "config.h"
#include "pid.h"


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
  
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);
  
    attachInterrupt(digitalPinToInterrupt(ENCA), Pid::readEncoder, RISING);

    Connection::setConnection();
}


void loop() {
    // Connection::receiveCommand(PID_CMD_SIZE, pid_callback);
    Pid::setPoseVariables(Connection::pose_msg);
    Connection::sendMessage(Connection::pose_msg, POSE_MSG_SIZE);
    Pid::pid();
    delay(100);


    if (digitalRead(BUTTON_PIN) == HIGH) {
        Pid::reset();
    }
}
