#include "../include/serial/serial.hpp"


// using namespace _serial;


bool _serial::openArduino() {
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


uint8_t _serial::crc8(const uint8_t* pocket, size_t size) {
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


void _serial::calcCommandCheckSum(Msg* msg) {
    uint8_t checksum = crc8(msg->msg(), msg->size() - 1);
    msg->set_checksum(checksum);
}


uint8_t _serial::calcMessageCheckSum(uint8_t buffer[], size_t size) {
    return crc8(buffer, size);
}


void serial::delay(size_t ms) {
    auto start_timer = std::chrono::system_clock::now();
    while (true) {
        auto end_timer = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
        if (dt >= (int)ms) {
            return;
        }
    }
}


void serial::sendCommand(Msg* msg) {
    _serial::calcCommandCheckSum(msg);
    write(_serial::Arduino, msg->msg(), msg->size());
    if (msg->size() != MsgSizes::PING && msg->task() == Tasks::PING) {
        std::cout << "WARNING: NON_PING command was sent with PING task" << std::endl;
    }
}


#define SIZE_FROM_SERIAL 0
#if not SIZE_FROM_SERIAL
Msg serial::receiveMessage(size_t size) {
    Msg msg(size);
    read(_serial::Arduino, msg.msg(), size);

    _serial::is_feedback_correct = false;

    if (msg[MsgStructure::START_BYTE0_IDX] == MsgStructure::START_BYTE && msg[MsgStructure::START_BYTE1_IDX] == MsgStructure::START_BYTE) {
        
        if (!_serial::calcMessageCheckSum(msg.msg(), size)) {
            _serial::is_feedback_correct = true;
            return msg;
        }
    }
    return Msg(0);
}


#else // receiveMessage with getting size from serial
Msg serial::receiveMessage(size_t size) {
    uint8_t st[2];
    uint8_t task;
    read(_serial::Arduino, st, 2);

    std::cout << (int)st[0] << " " << (int)st[1] << std::endl;

    Msg msg;

    if (st[0] == 64 && st[1] == 64) {
        std::cout << "OK" << std::endl;
        read(_serial::Arduino, &task, 1);
        std::cout << (int)task << std::endl;
        if (task == Tasks::PING) {
            std::cout << "TASK_OK" << std::endl;
        }
        msg = Msg(4);
        //msg.set_task(Tasks::PING);
        // uint8_t c;
        // read(Arduino, &c, 1);
        // msg.set_checksum(c);
        // msg.msg()[0] = 64;
        // msg.msg()[1] = 64;
    }

    _serial::is_feedback_correct = false;


    msg.print();

    // if (msg[MsgStructure::START_BYTE0_IDX] == MsgStructure::START_BYTE && msg[MsgStructure::START_BYTE1_IDX] == MsgStructure::START_BYTE) {
        
    //     if (!_serial::calcMessageCheckSum(msg.msg(), size)) {
    //         _serial::is_feedback_correct = true;
    //         std::cout << "OK" << std::endl;
    //         return msg;
    //     }
    // }
    return Msg(0);
}
#endif


bool serial::checkFeedback() {
    return _serial::is_feedback_correct;
}


bool serial::setConnection() {
    if (!_serial::openArduino()) {
        std::cout << "unable to connect" << std::endl;
        return false;
    }

    Msg ping_cmd(MsgSizes::PING);

    auto start_timer = std::chrono::system_clock::now();
    while (!_serial::is_feedback_correct) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(_serial::TIMER)) {
            sendCommand(&ping_cmd);
            receiveMessage(MsgSizes::PING);
            start_timer = std::chrono::system_clock::now();
        }
    }
    std::cout << "connected" << std::endl;
    delay(200);
    return true;
}


void serial::disconnectArduino() {
    Msg msg(MsgSizes::DISCONNECT);
    msg.set_task(Tasks::DISCONNECT);
    sendCommand(&msg);
    close(_serial::Arduino);
    std::cout << "disconnected" << std::endl;
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


int64_t serial::uint8arr_to_int64(uint8_t* data) {
    int64_t number = data[7];

    for (int i = 6; i >= 0; i--) {
        number <<= 8;
        number = number | data[i];
    }

    number = data[8] == 1 ? number : -number;
    
    return number;
}


void serial::float_to_uint8arr(float f, uint8_t* data) {
    memcpy(data, &f, sizeof(float));
}


float serial::uint8arr_to_float(uint8_t* data) {
    float f;
    memcpy(&f, data, sizeof(float));
    return f;
}
