#ifndef connection_h
#define connection_h


#define SERIAL_BAUDRATE 9600
#define START_BYTE        64
#define START_BYTE0_CELL   0
#define START_BYTE1_CELL   1

#define PING_CMD_SIZE      3
#define PING_MSG_SIZE      3


using callback = void (*) ();


namespace Connection {
    uint8_t* ping_msg = new uint8_t[PING_MSG_SIZE];
  
    uint8_t crc8(uint8_t data[], int size);
    uint8_t calcCommandCheckSum(uint8_t* command, uint64_t size);
    uint8_t calcMessageCheckSum(uint8_t* message, uint64_t size);
    void sendMessage(uint8_t* message, uint64_t size);
    bool receiveCommand(uint64_t size, callback cb);
    void setConnection();

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
    if (Serial.available() >= size) {
        command[START_BYTE0_CELL] = Serial.read();
        command[START_BYTE1_CELL] = Serial.read();
        if (command[START_BYTE0_CELL] == START_BYTE && command[START_BYTE1_CELL] == START_BYTE) {
            for (int idx = START_BYTE1_CELL + 1; idx < size; idx++) {
                command[idx] = Serial.read();
            }
            if (!calcCommandCheckSum(command, size)) {
                cb();
                return true;
            }
        }
    }
    return false;
}


void Connection::setConnection() {
    while (true) {
      if (receiveCommand(PING_CMD_SIZE, ping_callback)) {
          delete[] ping_msg;
          return;
      }
    }
}


#endif // connection_h
