#include "../include/serial/connect.hpp"


bool Connect::openArduino() {
    if (Arduino == -1) {
        return false;
    }

    tcgetattr(Arduino, &SerialPortSettings);

    SerialPortSettings.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;     // 8 bit chars
    SerialPortSettings.c_cflag &= ~(PARENB | PARODD);  // shut off parody
    SerialPortSettings.c_cflag &= ~CSTOPB; //no scts stop
    SerialPortSettings.c_iflag &= ~IGNBRK; //disable break processing
    SerialPortSettings.c_iflag = 0;        // no echo
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
    SerialPortSettings.c_oflag = 0;        // no remapping
    SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG |IEXTEN);
    SerialPortSettings.c_cc[VMIN] = 0;     // read doesn't block
    SerialPortSettings.c_cc[VTIME] = 0;    // 0s read timeout

    cfsetspeed(&SerialPortSettings, SERIAL_BAUDRATE);

    tcsetattr(Arduino, TCSANOW, &SerialPortSettings);

    return true;
}


uint8_t Connect::crc8(const uint8_t pocket[], size_t size) {
    uint8_t BYTE_SIZE = 8;
    uint8_t MSB_MASK = 0x80;
    uint8_t byte;
    uint8_t POLY = 0x7;
    uint8_t crc8 = 0xFF;

    for (size_t cell = 0; cell < size; cell++) {

        byte = pocket[cell];
        crc8 = crc8 ^ byte;

        for (int byte_number = 0; byte_number < BYTE_SIZE; byte_number++) {

            if (crc8 & MSB_MASK) {
                crc8 = (crc8 << 1) ^ POLY;
            }
            else {
                crc8 = crc8 << 1;
            }
        }
    }
    return crc8;
}


void Connect::calcCommandCheckSum(Msg* msg) {
    uint8_t checksum = crc8(msg->msg(), msg->size() - 1);
    msg->set_checksum(checksum);
}


uint8_t Connect::calcMessageCheckSum(uint8_t buffer[], size_t size) {
    return crc8(buffer, size);
}


void Connect::sendCommand(Msg* msg) {
    calcCommandCheckSum(msg);
    write(Arduino, msg->msg(), msg->size());
}


Msg Connect::receiveMessage(size_t size) {
    uint8_t* buf = new uint8_t[size];
    read(Arduino, buf, size);

    is_feedback_correct = false;

    if (buf[MsgStructure::START_BYTE0_IDX] == MsgStructure::START_BYTE && buf[MsgStructure::START_BYTE1_IDX] == MsgStructure::START_BYTE) {
        
        if (!calcMessageCheckSum(buf, size)) {
            is_feedback_correct = true;
            return Msg(size, buf);
        }
        
    }
    delete[] buf;
    return Msg(0, {});
}


bool Connect::checkFeedback() {
    return is_feedback_correct;
}


void Connect::delay(size_t ms) {
    auto start_timer = std::chrono::system_clock::now();
    while (true) {
        auto end_timer = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
        if (dt >= (int)ms) {
            return;
        }
    }
}


bool Connect::setConnection() {
    if (!openArduino()) {
        std::cout << "Unable to connect" << std::endl;
        return false;
    }

    Msg ping_cmd(PING_CMD_SIZE);

    auto start_timer = std::chrono::system_clock::now();
    while (!is_feedback_correct) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(TIMER)) {
            sendCommand(&ping_cmd);
            receiveMessage(PING_MSG_SIZE);
            start_timer = std::chrono::system_clock::now();
        }
    }
    std::cout << "connected" << std::endl;
    sleep(1);
    return true;
}


void Connect::disconnectArduino() {
    close(Arduino);
}
