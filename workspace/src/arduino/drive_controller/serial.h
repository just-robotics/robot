#ifndef DRIVE_CONTROLLER_SERIAL_H
#define DRIVE_CONTROLLER_SERIAL_H


#include "config.h"


using callback = void (*) (uint8_t* data);


namespace serial {
    callback cb;
    uint8_t cmd[CMD_SIZE];
    bool is_connected;

    void set_callbacks();
    void init();

    uint8_t crc8(uint8_t* data, int size);
    bool calcCheckSumForReceive();
    uint8_t calcCheckSumForSend(uint8_t* msg, uint64_t size);

    template<typename T>
    void num2arr(T number, uint8_t* output);
    template<typename T>
    T arr2num(uint8_t* data);
    
    bool receive();
    void send(uint8_t* msg, uint64_t size);

    void connect();

    void send_data(int64_t* poses, float* vels, float* targets);
}


void serial::init() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);
    cb = [] (uint8_t* msg) {send(msg, MSG_SIZE);};
}


uint8_t serial::crc8(uint8_t* data, int size) {
    uint8_t byte;
    uint8_t POLY = 0x7;
    uint8_t crc8 = 0xFF;

    for (int j = 0; j < size; j++) {

        byte = data[j];
        crc8 = crc8 ^ byte;

        for (int i = 0; i < 8; i++) {

            if (crc8 & 0x80) {
                crc8 = (crc8 << 1) ^ POLY;
            }
            else {
                crc8 = crc8 << 1;
            }
        }
    }
    return crc8;
}


bool serial::calcCheckSumForReceive() {
    return crc8(cmd, CMD_SIZE) == 0;
}


uint8_t serial::calcCheckSumForSend(uint8_t* msg, uint64_t size) {
    return crc8(msg, size - 1);
}


template<typename T>
void serial::num2arr(T number, uint8_t* output) {
    memcpy(output, &number, sizeof(T));
}


template<typename T>
T serial::arr2num(uint8_t* data) {
    T number;
    memcpy(&number, data, sizeof(T));
    return number;
}


bool serial::receive() {
    if (Serial.available() >= CMD_SIZE) {
        cmd[START_BYTE0_IDX] = Serial.read();
        cmd[START_BYTE1_IDX] = Serial.read();
        if (cmd[START_BYTE0_IDX] == START_BYTE && cmd[START_BYTE1_IDX] == START_BYTE) {
            for (int idx = DATA_IDX; idx < CMD_SIZE; idx++) {
                cmd[idx] = Serial.read();
            }
            if (calcCheckSumForReceive()) {
                cb(cmd);
                return true;
            }
        }
    }
    return false;
}


void serial::send(uint8_t* msg, uint64_t size) {
    msg[START_BYTE0_IDX] = START_BYTE;
    msg[START_BYTE1_IDX] = START_BYTE;
    msg[size-1] = calcCheckSumForSend(msg, size);
    Serial.write(msg, size);
}


void serial::connect() {
    while (true) {
        if (receive()) {
            set_callbacks();
            return;
        }
        delay(1000);
    }
}


void serial::send_data(int64_t* poses, float* vels, float* targets) {
    uint8_t data[MSG_SIZE];
    num2arr(poses[0], data + POSE0_IDX);
    num2arr(poses[1], data + POSE1_IDX);

    num2arr(vels[0], data + VEL0_IDX);
    num2arr(vels[1], data + VEL1_IDX);

    num2arr(targets[0], data + TARGET0_IDX);
    num2arr(targets[1], data + TARGET1_IDX);
    
    send(data, MSG_SIZE);
}


#endif // DRIVE_CONTROLLER_SERIAL_H
