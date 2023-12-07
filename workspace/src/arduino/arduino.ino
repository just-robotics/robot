
#include "bot.h"
#include "connection.h"
#include "config.h"


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Bot::init();
}


void loop() {
    Connection::receiveCommand();
}
