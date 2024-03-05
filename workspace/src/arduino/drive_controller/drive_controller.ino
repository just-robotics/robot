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

    motor0.setPwm(1, 255);
    motor0.setPwm(1, 255);
    motor0.setPwm(1, 255);
    motor0.setPwm(1, 255);
}


void loop() {
    serial::receive();
    
    Motor::spinMotors();

    if (millis() - start_time >= 100) {
        serial::send_data(poses, velocities, targets);
        start_time = millis();
    }
}
