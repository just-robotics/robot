#ifndef SERIAL_CONNECT_HPP
#define SERIAL_CONNECT_HPP


#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>


#include "msg.hpp"


class Connect {
private:
    static const uint8_t TIMER = 200;

    static const uint8_t PING_MSG_SIZE = 3;
    static const uint8_t PING_CMD_SIZE = 3;

    inline static int Arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    inline static bool is_feedback_correct = false;

private:
    static bool openArduino();

public:
    static bool setConnection();
    static void disconnectArduino();

private:
    static uint8_t crc8(const uint8_t pocket[], uint64_t size);
    static void calcCommandCheckSum(Msg* msg);
    static uint8_t calcMessageCheckSum(uint8_t buffer[], size_t size);

public:
    static void sendCommand(Msg* msg);

public:
    static Msg receiveMessage(size_t size);
};


inline struct termios SerialPortSettings;


#endif // SERIAL_CONNECT_HPP
