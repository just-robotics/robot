#ifndef SERIAL_SERIAL_HPP
#define SERIAL_SERIAL_HPP


#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "msg.hpp"


namespace _serial {
    const size_t SERIAL_BAUDRATE = 2000000;
    const uint8_t TIMER = 200;

    inline const int Arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    inline bool is_feedback_correct = false;

    bool openArduino();

    uint8_t crc8(const uint8_t* pocket, uint64_t size);
    void calcCommandCheckSum(Msg* msg);
    uint8_t calcMessageCheckSum(uint8_t buffer[], size_t size);

    inline struct termios SerialPortSettings;
}


namespace serial {
    void delay(size_t ms);

    bool setConnection();
    void disconnectArduino();

    void sendCommand(Msg* msg);
    Msg receiveMessage(size_t size);
    bool checkFeedback();

    void int64_to_uint8arr(int64_t number, uint8_t* output);
    int64_t uint8arr_to_int64(uint8_t* data);
    void float_to_uint8arr(float val, uint8_t* data);
    float uint8arr_to_float(uint8_t* data);
}


#endif // SERIAL_SERIAL_HPP
