#ifndef SERIAL_CONNECT_HPP
#define SERIAL_CONNECT_HPP


#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "msg.hpp"


#define POSE_IDX       2
#define TARGET_IDX    11
#define U_IDX         20

#define POSE_SIZE      9
#define TARGET_SIZE    9
#define U_SIZE         9

#define POSE_MSG_SIZE 30


class Connect {
private:
    static const size_t SERIAL_BAUDRATE = 2000000;
    static const uint8_t TIMER = 200;

    static const uint8_t PING_MSG_SIZE = 3;
    static const uint8_t PING_CMD_SIZE = 3;

    inline static int Arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    inline static bool is_feedback_correct = false;

private:
    static bool openArduino();

public:
    static void delay(size_t ms);
    static bool setConnection();
    static void disconnectArduino();

private:
    static uint8_t crc8(const uint8_t* pocket, uint64_t size);
    static void calcCommandCheckSum(Msg* msg);
    static uint8_t calcMessageCheckSum(uint8_t buffer[], size_t size);

public:
    static void sendCommand(Msg* msg);

public:
    static Msg receiveMessage(size_t size);
    static bool checkFeedback();

    static void int64_to_uint8arr(int64_t number, uint8_t* output);
    static int64_t uint8arr_to_int64(uint8_t* data);
    static void float_to_uint8arr(float val, uint8_t* data);
    static float uint8arr_to_float(uint8_t* data);
};


inline struct termios SerialPortSettings;


#endif // SERIAL_CONNECT_HPP
