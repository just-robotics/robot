#include "connection.h"


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Connection::setConnection();
}


void loop() {
    
}
