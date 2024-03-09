#include "serial.h"
#include "config.h"
#include "motor.h"


uint64_t start_time;


void serial::set_callbacks() {
    cb = Motor::callback;
}


void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Motor::init();

    serial::init();

    serial::connect();

    start_time = millis();
}


void loop() {
    serial::receive();
    
    Motor::spinMotors();

    if (millis() - start_time >= 100) {
        serial::send_data(msg_poses, msg_vels, msg_targets);
        start_time = millis();
    }
}
