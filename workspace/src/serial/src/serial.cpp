#include "../include/serial/serial.hpp"


Serial::Serial(std::string name, size_t baudrate, std::string topic) {
    fd_ = open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    tcgetattr(fd_, &serial_port_settings_);

    serial_port_settings_.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
    serial_port_settings_.c_cflag &= ~CSIZE;
    serial_port_settings_.c_cflag |= CS8;     // 8 bit chars
    serial_port_settings_.c_cflag &= ~(PARENB | PARODD);  // shut off parody
    serial_port_settings_.c_cflag &= ~CSTOPB; //no scts stop
    serial_port_settings_.c_iflag &= ~IGNBRK; //disable break processing
    serial_port_settings_.c_iflag = 0;        // no echo
    serial_port_settings_.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
    serial_port_settings_.c_oflag = 0;        // no remapping
    serial_port_settings_.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG |IEXTEN);
    serial_port_settings_.c_cc[VMIN] = 0;     // read doesn't block
    serial_port_settings_.c_cc[VTIME] = 0;    // 0s read timeout

    cfsetspeed(&serial_port_settings_, baudrate);

    tcsetattr(fd_, TCSANOW, &serial_port_settings_);

    name_ = name;
    baudrate_ = baudrate;
    topic_ = topic;

    is_feedback_correct_ = false;
    if (fd_ != -1) {
        is_opened_ = true;
    }
}


Serial::~Serial() {
    disconnect();
}


uint8_t Serial::crc8(const uint8_t* pocket, size_t size) {
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


void Serial::setChecksumForSend(Msg* msg) {
    uint8_t checksum = crc8(msg->data(), msg->size() - 1);
    msg->setChecksum(checksum);
}


bool Serial::checkChecksumFromReceive(uint8_t buffer[], size_t size) {
    return crc8(buffer, size) == 0;
}


void Serial::delay(size_t ms) {
    auto start_timer = std::chrono::system_clock::now();
    while (true) {
        auto end_timer = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
        if (dt >= (int)ms) {
            return;
        }
    }
}


void Serial::send(Msg* msg) {
    setChecksumForSend(msg);
    msg->setStartBytes();
    std::cout << "msg ";
    msg->print();
    uint8_t buf[] = {64, 64, 0, 246};
    write(fd_, buf, 4);
    if (msg->size() != msg_sizes::PING && msg->task() == tasks::PING) {
        std::cout << "WARNING: NON_PING command was sent with PING task" << std::endl;
    }
}


Msg Serial::receive(size_t size) {
    Msg msg(size);
    //uint8_t* buf = new uint8_t[size];
    read(fd_, msg.data(), size);

    is_feedback_correct_ = false;

    // for (int i = 0; i < 4; i++) {
    //     std::cout << "buf[" << i << "] = " << buf[i];
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    msg.print();

    if (msg[msg_structure::START_BYTE0_IDX] == msg_structure::START_BYTE && msg[msg_structure::START_BYTE1_IDX] == msg_structure::START_BYTE) {
        
        if (checkChecksumFromReceive(msg.data(), size)) {
            is_feedback_correct_ = true;
            return msg;
        }
    }
    return Msg(0);
}


bool Serial::checkFeedback() {
    return is_feedback_correct_;
}


bool Serial::is_opened() {
    std::cout << "fd = " << fd_ << std::endl;
    return (fd_ != -1 && is_opened_);
}


bool Serial::connect() {
    if (!is_opened()) {
        std::cout << "not opened" << std::endl;
        return false;
    }

    Msg ping_cmd(msg_sizes::PING);
    
    auto start_timer = std::chrono::system_clock::now();
    while (!is_feedback_correct_) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(CONNECT_TIMER_)) {
            send(&ping_cmd);
            receive(msg_sizes::PING);
            start_timer = std::chrono::system_clock::now();
        }
    }
    std::cout << "connected" << std::endl;
    delay(100);
    return true;
}


void Serial::disconnect() {
    Msg msg(msg_sizes::DISCONNECT);
    msg.setTask(tasks::DISCONNECT);
    send(&msg);
    close(fd_);
    is_opened_ = false;
    std::cout << "disconnected" << std::endl;
}


void Serial::int64_to_uint8arr(int64_t number, uint8_t* output) {
    uint8_t byte = 0x000000FF;

    output[8] = number < 0 ? 0 : 1;
    uint64_t u_number = abs(number);

    output[0] = u_number & byte;
    
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        output[i] = u_number & byte;
    }
}


int64_t Serial::uint8arr_to_int64(uint8_t* data) {
    int64_t number = data[7];

    for (int i = 6; i >= 0; i--) {
        number <<= 8;
        number = number | data[i];
    }

    number = data[8] == 1 ? number : -number;
    
    return number;
}


void Serial::float_to_uint8arr(float f, uint8_t* data) {
    memcpy(data, &f, sizeof(float));
}


float Serial::uint8arr_to_float(uint8_t* data) {
    float f;
    memcpy(&f, data, sizeof(float));
    return f;
}


std::string Serial::name() {
    return name_;
}


size_t Serial::baudrate() {
    return baudrate_;
}


std::string Serial::topic() {
    return topic_;
}
