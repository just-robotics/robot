#ifndef SERIAL_MSG_HPP
#define SERIAL_MSG_HPP


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>


class Msg {
private:
    uint8_t* data_;
    size_t size_;

public:
    Msg();
    Msg(size_t size, bool fill_with_zeros=true);
    Msg(size_t size, uint8_t* data);
    Msg(size_t size, const std::initializer_list<uint8_t> array);
    ~Msg();
    
    Msg(const Msg& other);
    Msg(Msg&& other);

    Msg& operator=(const Msg& other);
    Msg& operator=(Msg&& other);
    uint8_t operator[](size_t idx);

    void setStartBytes();
    void setChecksum(uint8_t checksum);
    void setTask(uint8_t task);

    uint8_t* data();
    size_t size();
    uint8_t task();

    void print();
};


namespace msg_structure {
    const uint8_t START_BYTE = 64;
    const size_t START_BYTE0_IDX = 0;
    const size_t START_BYTE1_IDX = 1;
    const size_t TASK_IDX = 2;
}


namespace tasks {
    const uint8_t PING = 0;
    const uint8_t DISCONNECT = 1;
    const uint8_t SET_PID = 2;
}


namespace msg_sizes {
    const uint8_t PING = 4;
    const uint8_t DISCONNECT = 4;
    const uint8_t SET_PID = 16;
    const uint8_t RESET_PID = 4;
    const uint8_t OK = 4;
    const uint8_t POSE = 40;

    inline std::vector<size_t> msg_sizes = {
        PING,
        DISCONNECT,
    };
}


namespace serial_msgs {
    inline std::vector<size_t> pose_idx = {3, 12, 21, 30};
    inline std::vector<size_t> pose_sz = {9, 9, 9, 9};

    inline std::vector<size_t> pid_idx = {3, 7, 11};
    inline std::vector<size_t> pid_sz = {4, 4, 4};
}


#endif // SERIAL_MSG_HPP
