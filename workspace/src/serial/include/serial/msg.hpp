#ifndef SERIAL_MSG_HPP
#define SERIAL_MSG_HPP


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>


namespace MsgStructure {
    const uint8_t START_BYTE = 64;
    const uint8_t START_BYTE0_IDX = 0;
    const uint8_t START_BYTE1_IDX = 1;
}


class Msg {
private:
    size_t size_;
    uint8_t* msg_;

public:
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

    size_t size();
    uint8_t checksum();
    uint8_t* msg();

    void print(bool is_int=true);
};


namespace Msgs {
    inline std::vector<size_t> pose_target_idx = {2, 11, 20};
    inline std::vector<size_t> pose_target_sz = {9, 9, 9};
}


#endif // SERIAL_MSG_HPP
