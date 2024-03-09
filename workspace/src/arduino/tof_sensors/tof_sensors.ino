#include <Wire.h>
#include <VL53L1X.h>

#include "serial.h"
#include "config.h"


VL53L1X sensors[TOF_NUM];

int xshuts[TOF_NUM];
int addr[TOF_NUM];

float data[TOF_NUM];
uint8_t msg[MSG_SIZE];

bool is_tofs_found = false;

uint64_t prev_time = 0;


void nop() {
    return;
}

void serial::set_callbacks() {
    cb = nop;
}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // set low to xshut pins
    for (uint8_t i = 0; i < TOF_NUM; i++) {
        xshuts[i] = XSHUTS_START_PIN + i;
        addr[i] = START_ADDR + i;
        pinMode(xshuts[i], OUTPUT);
        digitalWrite(xshuts[i], LOW);
    }

#if SERIAL
    serial::init();
    serial::connect();
#endif

    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(0x28);
    Serial.begin (SERIAL_BAUDRATE);

    for (uint8_t i = 0; i < TOF_NUM; i++) {
        digitalWrite(xshuts[i], HIGH);
        delay(10);
        sensors[i].init();
        sensors[i].setAddress(addr[i]);
    }

#if !SERIAL
    Serial.println();
    Serial.println("Addresses set");
#endif

    for (uint8_t i = 0; i < TOF_NUM; i++) {
        sensors[i].setDistanceMode(VL53L1X::Long);
        sensors[i].setMeasurementTimingBudget(50000);
        sensors[i].startContinuous(50);
        sensors[i].setTimeout(500);
    }

    Serial.println ("I2C scanner. Scanning ...");
    int tofs = 0;


    for (int i = 1; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
#if !SERIAL
            Serial.print ("Found address: ");
            Serial.print (i, DEC);
            Serial.print (" (0x");
            Serial.print (i, HEX);
            Serial.println (")");
#endif
            tofs++;
            delay (1);
        }
    }
#if !SERIAL
    Serial.println ("Done.");
    Serial.print ("Found ");
    Serial.print (tofs, DEC);
    Serial.println (" device(s).");
#else
    prev_time = millis();
#endif
    is_tofs_found = tofs ? true : false;
}

int64_t t__ = 0;
void loop() {
    Serial.println((int)(millis() - t__));
    t__ = millis();
    if (is_tofs_found) {
        for (int i = 0; i < TOF_NUM; i++) {
            sensors[i].read();
            data[i] = sensors[i].ranging_data.range_mm;
#if !SERIAL
            Serial.print(data[i]);
            Serial.print(" ");
#endif
        }
#if !SERIAL
        Serial.println();
#else
        memset(msg, -1, MSG_SIZE);
        for (int i = 0; i < TOF_NUM; i++) {
            serial::num2arr(data[i] / 1000, msg + DATA_IDX + DATA_SIZE * i);
        }

        int64_t curr_time = millis();
        if (abs(curr_time - prev_time) > TIMER) {
            serial::send(msg, MSG_SIZE);
            prev_time = curr_time;
        }
#endif
    }
}
