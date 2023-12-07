
#ifndef Bot_h
#define Bot_h


#include "config.h"


namespace Bot {
    static void init();
    static void moveForward(uint8_t speed);
    static void moveBackward(uint8_t speed);
    static void stop();
    static void turnRight(uint8_t speed);
    static void turnLeft(uint8_t speed);
    static void ledOn();
    static void ledOff();
};


void Bot::init() {
    pinMode(M1_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
}


void Bot::moveForward(uint8_t speed) {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::moveBackward(uint8_t speed) {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::stop() {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}


void Bot::turnRight(uint8_t speed) {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::turnLeft(uint8_t speed) {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::ledOn() {
    digitalWrite(LED_BUILTIN, HIGH);
}


void Bot::ledOff() {
    digitalWrite(LED_BUILTIN, LOW);
}


#endif
