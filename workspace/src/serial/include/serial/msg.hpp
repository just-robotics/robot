#ifndef SERIAL_MSG_HPP
#define SERIAL_MSG_HPP


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>


class Msg {
private:
    size_t size_;
    uint8_t* msg_;
    uint8_t task_;

public:
    Msg();
    Msg(size_t size);
    Msg(size_t size, const std::initializer_list<uint8_t> array);
    Msg(size_t size, const uint8_t* array);
    ~Msg();

    Msg(const Msg& other);
    Msg(Msg&& other);

    Msg& operator=(const Msg& other);
    Msg& operator=(Msg&& other);
    uint8_t operator[](size_t idx);

    void set_checksum(uint8_t checksum);
    void set_task(uint8_t task);

    size_t size();
    uint8_t checksum();
    uint8_t* msg();
    uint8_t task();

    void print();
};


namespace MsgStructure {
    const uint8_t START_BYTE = 64;
    const size_t START_BYTE0_IDX = 0;
    const size_t START_BYTE1_IDX = 1;
    const size_t TASK_IDX = 2;
    const size_t ANSWER_IDX = TASK_IDX;
}



#define OK_ANSWER_TASK        5
#define POSE_ANSWER_TASK      6


namespace Tasks {
    const uint8_t PING = 0;
    const uint8_t DISCONNECT = 1;
    const uint8_t SET_PID = 2;
    const uint8_t RESET_PID = 3;
    const uint8_t OK = 4;
    const uint8_t POSE = 5;
}


namespace MsgSizes {
    const uint8_t PING = 4;
    const uint8_t DISCONNECT = 4;
    const uint8_t SET_PID = 16;
    const uint8_t RESET_PID = 4;
    const uint8_t OK = 4;
    const uint8_t POSE = 40;

    inline std::vector<size_t> msg_sizes = {
        PING,
        DISCONNECT,
        SET_PID,
        RESET_PID,
        OK,
        POSE,
    };
}


namespace Msgs {
    inline std::vector<size_t> pose_idx = {3, 12, 21, 30};
    inline std::vector<size_t> pose_sz = {9, 9, 9, 9};

    inline std::vector<size_t> pid_idx = {3, 7, 11};
    inline std::vector<size_t> pid_sz = {4, 4, 4};
}


#endif // SERIAL_MSG_HPP
