#ifndef PID_REGULATOR_SERIAL_H
#define PID_REGULATOR_SERIAL_H


#include "config.h"


using callback = void (*) (uint8_t* data);


namespace serial {
    callback* callbacks;
    uint64_t* msg_sizes;
    //uint8_t pose_msg[POSE_MSG_SIZE];
    bool is_connected;

    void set_callbacks();
    void set_msg_sizes();
    void init();

    uint8_t crc8(uint8_t* data, int size);
    bool calcCheckSumForReceive(uint8_t* msg, uint64_t size);
    uint8_t calcCheckSumForSend(uint8_t* msg, uint64_t size);

    void int64_to_uint8arr(int64_t number, uint8_t* output);
    float uint8arr_to_float(uint8_t* data);
    
    bool receive();
    void send(uint8_t* msg, uint64_t size, uint8_t task);

    void connect();

    void send_poses(int64_t* poses);
}


void serial::init() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);
    set_callbacks();
    set_msg_sizes();
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


bool serial::calcCheckSumForReceive(uint8_t* msg, uint64_t size) {
    return crc8(msg, size) == 0;
}


uint8_t serial::calcCheckSumForSend(uint8_t* msg, uint64_t size) {
    return crc8(msg, size - 1);
}


void serial::int64_to_uint8arr(int64_t number, uint8_t* output) {
    uint8_t byte = 0x000000FF;

    output[8] = number < 0 ? 0 : 1;
    uint64_t u_number = abs(number);

    output[0] = u_number & byte;
    
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        output[i] = u_number & byte;
    }
}


float serial::uint8arr_to_float(uint8_t* data) {
    float f;
    memcpy(&f, data, sizeof(float));
    return f;
}


bool serial::receive() {
    uint64_t size = 0;
    if (Serial.available() > 0) {
        uint8_t sb0 = Serial.read();
        uint8_t sb1 = Serial.read();
        if (sb0 == START_BYTE && sb1 == START_BYTE) {
            uint8_t task = Serial.read();
            size = 4;//msg_sizes[task];
            uint8_t msg[size];
            msg[START_BYTE0_IDX] = sb0;
            msg[START_BYTE1_IDX] = sb1;
            msg[TASK_IDX] = task;
            for (int idx = TASK_IDX + 1; idx < size; idx++) {
                msg[idx] = Serial.read();
            }
            if (calcCheckSumForReceive(msg, size)) {
                callbacks[0](msg);
                return true;
            }
        }
    }
    return false;
}


void serial::send(uint8_t* msg, uint64_t size, uint8_t task) {
    msg[START_BYTE0_IDX] = START_BYTE;
    msg[START_BYTE1_IDX] = START_BYTE;
    msg[TASK_IDX] = task;
    msg[size-1] = calcCheckSumForSend(msg, size);
    Serial.write(msg, size);
}


void serial::connect() {
    while (true) {
        if (receive()) {
          is_connected = true;
          return; 
        }
        delay(1000);
    }
}


void serial::send_poses(int64_t* poses) {
    uint8_t data[POSE_MSG_SIZE];
    int64_to_uint8arr(poses[0], data + POSE0_IDX);
    int64_to_uint8arr(poses[1], data + POSE1_IDX);
    int64_to_uint8arr(poses[2], data + POSE2_IDX);
    int64_to_uint8arr(poses[3], data + POSE3_IDX);
    send(data, POSE_MSG_SIZE, POSE_TASK);
}


#endif // PID_REGULATOR_SERIAL_H
