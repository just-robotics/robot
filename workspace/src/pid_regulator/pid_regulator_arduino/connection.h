#ifndef PID_REGULATOR_CONNECTION_H
#define PID_REGULATOR_CONNECTION_H


#include "config.h"


using callback = void (*) (uint8_t* data, uint64_t size);


namespace Connection {
    uint8_t ping_msg[PING_SIZE];
    uint8_t pose_msg[POSE_MSG_SIZE];

    void init();
    uint8_t crc8(uint8_t data[], int size);
    uint8_t calcCommandCheckSum(uint8_t* command, uint64_t size);
    uint8_t calcMessageCheckSum(uint8_t* message, uint64_t size);
    void sendMessage(uint8_t* message, uint64_t size, uint8_t answer);
    bool receiveCommand(uint64_t size, callback cb);
    void setConnection();
    void int64_to_uint8arr(int64_t number, uint8_t* output);
    float uint8arr_to_float(uint8_t* data);
    callback* callbacks;
    uint64_t* cmd_sizes;
    bool is_connected;
    void send_poses(int64_t* poses);
}


uint8_t Connection::crc8(uint8_t data[], int size) {
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


uint8_t Connection::calcCommandCheckSum(uint8_t* command, uint64_t size) {
    return crc8(command, size);
}


uint8_t Connection::calcMessageCheckSum(uint8_t* message, uint64_t size) {
    return crc8(message, size - 1);
}


void Connection::sendMessage(uint8_t* message, uint64_t size, uint8_t answer) {
    message[START_BYTE0_CELL] = START_BYTE;
    message[START_BYTE1_CELL] = START_BYTE;
    message[ANSWER_CELL] = answer;
    message[size-1] = calcMessageCheckSum(message, size);

    for (int cell = START_BYTE0_CELL; cell < size; cell++) {
        Serial.print(char(message[cell]));
    }
}


bool Connection::receiveCommand(uint64_t size, callback cb) {
    uint8_t sb0 = 0;
    uint8_t sb1 = 0;
    if (Serial.available() > 0) {
        sb0 = Serial.read();
        sb1 = Serial.read();
        if (sb0 == START_BYTE && sb1 == START_BYTE) {
            uint8_t task = Serial.read();
            uint64_t sz = cmd_sizes[task];
            uint8_t command[sz];
            command[START_BYTE0_CELL] = START_BYTE;
            command[START_BYTE1_CELL] = START_BYTE;
            command[TASK_CELL] = task;
            for (int idx = TASK_CELL + 1; idx < sz; idx++) {
                command[idx] = Serial.read();
            }
            if (!calcCommandCheckSum(command, sz)) {
                if (task != PING_TASK) {
                    callbacks[task](command, sz);
                }
                else {
                    cb(command, sz);
                }
                return true;
            }
        }
    }
    return false;
}


void Connection::setConnection() {
    while (true) {
        if (receiveCommand(PING_SIZE, Connection::callbacks[PING_TASK])) {
          is_connected = true;
          return; 
        }
    }
}


void Connection::int64_to_uint8arr(int64_t number, uint8_t* output) {
    uint8_t byte = 0x000000FF;

    output[8] = number < 0 ? 0 : 1;
    uint64_t u_number = abs(number);

    output[0] = u_number & byte;
    
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        output[i] = u_number & byte;
    }
}


float Connection::uint8arr_to_float(uint8_t* data) {
    float f;
    memcpy(&f, data, sizeof(float));
    return f;
}


void Connection::send_poses(int64_t* poses) {
    uint8_t data[POSE_MSG_SIZE];
    int64_to_uint8arr(poses[0], data + POSE0_IDX);
    int64_to_uint8arr(poses[1], data + POSE1_IDX);
    int64_to_uint8arr(poses[2], data + POSE2_IDX);
    int64_to_uint8arr(poses[3], data + POSE3_IDX);
    sendMessage(data, POSE_MSG_SIZE, POSE_TASK);
}


#endif // PID_REGULATOR_CONNECTION_H
