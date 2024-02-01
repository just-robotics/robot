#ifndef ULTRASOUND_SENSORS_ULTRASOUND_H
#define ULTRASOUND_SENSORS_ULTRASOUND_H


#include "config.h"


class Ultrasound {
private:
    uint8_t trig_;
    uint8_t echo_;
    
    float distance_;

public:
    Ultrasound(uint8_t trig, uint8_t echo);
    ~Ultrasound() = default;

    float get_distance();
};


Ultrasound sensor0(PIN_TRIG_0, PIN_ECHO_0);
Ultrasound sensor1(PIN_TRIG_1, PIN_ECHO_1);
Ultrasound sensor2(PIN_TRIG_2, PIN_ECHO_2);
Ultrasound sensor3(PIN_TRIG_3, PIN_ECHO_3);
Ultrasound sensor4(PIN_TRIG_4, PIN_ECHO_4);
Ultrasound sensor5(PIN_TRIG_5, PIN_ECHO_5);
Ultrasound sensor6(PIN_TRIG_6, PIN_ECHO_6);
Ultrasound sensor7(PIN_TRIG_7, PIN_ECHO_7);
Ultrasound sensor8(PIN_TRIG_8, PIN_ECHO_8);
Ultrasound sensor9(PIN_TRIG_9, PIN_ECHO_9);
Ultrasound sensor10(PIN_TRIG_10, PIN_ECHO_10);
Ultrasound sensor11(PIN_TRIG_11, PIN_ECHO_11);


Ultrasound::Ultrasound(uint8_t trig, uint8_t echo) {
    trig_ = trig;
    echo_ = echo;

    pinMode(trig_, OUTPUT);
    pinMode(echo_, INPUT);

    distance_ = 0;
}


float Ultrasound::get_distance() {
    digitalWrite(trig_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_, LOW);
    delayMicroseconds(10);
    
    distance_ = (pulseIn(echo_, HIGH) / 2) / 29.1;

    return distance_;
}


#endif // SENSORS_ULTRASONIC_H
