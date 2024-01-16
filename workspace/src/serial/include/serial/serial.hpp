#ifndef SERIAL_SERIAL_HPP
#define SERIAL_SERIAL_HPP


#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <iostream>
#include <string>

#include "msg.hpp"


class Serial {
private:
    static const size_t CONNECT_TIMER_ = 1000;

    struct termios serial_port_settings_;

    std::string name_;
    size_t baudrate_;
    std::string topic_;
    int fd_;

    bool is_opened_;
    bool is_feedback_correct_;

public:
    Serial(std::string name, size_t baudrate, std::string topic);
    ~Serial();
    Serial(const Serial& other) = delete;
    Serial(Serial&& other) = delete;

    Serial* operator=(const Serial& other) = delete;
    Serial* operator=( Serial&& other) = delete;

private:
    static uint8_t crc8(const uint8_t* data, uint64_t size);
    static void setChecksumForSend(Msg* msg);
    static bool checkChecksumFromReceive(uint8_t buffer[], size_t size);

    static void delay(size_t ms);

public:
    void send(Msg* msg);
    Msg receive(size_t size);

    bool checkFeedback();
    bool is_opened();
    bool connect();
    void disconnect();

    static void int64_to_uint8arr(int64_t number, uint8_t* output);
    static int64_t uint8arr_to_int64(uint8_t* data);
    static void float_to_uint8arr(float val, uint8_t* data);
    static float uint8arr_to_float(uint8_t* data);

    std::string name();
    size_t baudrate();
    std::string topic();
};


#endif // SERIAL_SERIAL_HPP
