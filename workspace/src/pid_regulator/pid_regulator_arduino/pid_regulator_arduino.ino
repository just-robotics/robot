#include "connection.h"
#include "config.h"
#include "pid.h"


void init_callbacks() {
    Connection::callbacks = new callback[CALLBACKS_CNT];

    Connection::callbacks[PING_CALLBACK] = [] () {Connection::sendMessage(Connection::ping_msg, PING_MSG_SIZE);};
    Connection::callbacks[PID_CALLBACK] = [] (uint8_t* data) {Pid::setPidVariables(data);};
    Connection::callbacks[RESET_CALLBACK] = [] (uint8_t* data) {Pid::reset();};
}


uint64_t start_time;


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
  
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);
  
    attachInterrupt(digitalPinToInterrupt(ENCA), Pid::readEncoder, RISING);

    init_callbacks();

    Connection::setConnection();

    Blink::blink_timer = millis();

    start_time = millis();
}


void loop() {  
    Connection::receiveCommand(PID_CMD_SIZE, Connection::callbacks[PID_CALLBACK]);
    Pid::pid();
    
    if (millis() - start_time >= 100) {
        Pid::setPoseVariables(Connection::pose_msg);
        Connection::sendMessage(Connection::pose_msg, POSE_MSG_SIZE);
        start_time = millis();
    }

    Blink::checkBlink();
}
