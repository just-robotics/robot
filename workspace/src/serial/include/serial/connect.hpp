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


#define MAX_SIZE 38
#define MESSAGE_SIZE 4
#define COMMAND_SIZE 6
#define TIMER 200
#define MESSAGE_START_BYTE1_CELL 0
#define MESSAGE_START_BYTE2_CELL 1
#define MESSAGE_ANSWER_CELL      2
#define MESSAGE_CHECKSUM_CELL    3


class Connect {
public:
    static void init();

private:
    inline static int Arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    inline static uint8_t message[MESSAGE_SIZE];
    inline static bool is_feedback_correct = false;

private:
    static bool openArduino();

public:
    static bool setConnection();
    static void disconnectArduino();

private:
    static uint8_t crc8(const uint8_t pocket[], uint64_t size);
    static void calcCommandCheckSum(Msg* msg);
    static uint8_t calcMessageCheckSum(uint8_t buffer[]);

public:
    static void sendCommand(Msg* msg);

public:
    static Msg receiveMessage();
};


inline struct termios SerialPortSettings;


#endif // SERIAL_CONNECT_HPP
