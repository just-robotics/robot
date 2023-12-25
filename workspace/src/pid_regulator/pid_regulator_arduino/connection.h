#ifndef PID_REGULATOR_CONNECTION_H
#define PID_REGULATOR_CONNECTION_H


#include "config.h"


using callback = void (*) (uint8_t* data, uint64_t size);


namespace Connection {
    uint8_t ping_msg[PING_MSG_SIZE];
    uint8_t pose_msg[POSE_MSG_SIZE];

    uint8_t crc8(uint8_t data[], int size);
    uint8_t calcCommandCheckSum(uint8_t* command, uint64_t size);
    uint8_t calcMessageCheckSum(uint8_t* message, uint64_t size);
    void sendMessage(uint8_t* message, uint64_t size);
    bool receiveCommand(uint64_t size, callback cb);
    void setConnection();
    void int64_to_uint8arr(int64_t number, uint8_t* output);
    float uint8arr_to_float(uint8_t* data);

    callback ping_callback = [] () {sendMessage(ping_msg, PING_MSG_SIZE);};
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


void Connection::sendMessage(uint8_t* message, uint64_t size) {
    message[START_BYTE0_CELL] = START_BYTE;
    message[START_BYTE1_CELL] = START_BYTE;
    message[size-1] = calcMessageCheckSum(message, size);

    for (int cell = START_BYTE0_CELL; cell < size; cell++) {
        Serial.print(char(message[cell]));
    }
}


bool Connection::receiveCommand(uint64_t size, callback cb) {
    uint8_t command[size];
    if (Serial.available() > size) {
        command[START_BYTE0_CELL] = Serial.read();
        command[START_BYTE1_CELL] = Serial.read();
        if (command[START_BYTE0_CELL] == START_BYTE && command[START_BYTE1_CELL] == START_BYTE) {
            for (int idx = START_BYTE1_CELL + 1; idx < size; idx++) {
                command[idx] = Serial.read();
            }
            if (!calcCommandCheckSum(command, size)) {
                cb(command, size);
                return true;
            }
        }
    }
    return false;
}


void Connection::setConnection() {
    while (true) {
      if (receiveCommand(PING_CMD_SIZE, ping_callback)) {
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
    union {
      float float_variable;
      uint8_t uint8_array[4];
    } un;

    memcpy(un.uint8_array, data, 4);
    return un.float_variable;
}


#endif // PID_REGULATOR_CONNECTION_H
