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
    Msg(size_t size);
    Msg(size_t size, uint8_t* data);
    Msg(size_t size, const std::initializer_list<uint8_t> array);
    ~Msg();
    
    Msg(const Msg& other);
    Msg(Msg&& other);

    Msg& operator=(const Msg& other);
    Msg& operator=(Msg&& other);
    uint8_t& operator[](size_t idx);

    void setStartBytes();
    void setChecksum(uint8_t checksum);

    uint8_t* data();
    size_t size();

    void print();
};


namespace msg_structure {
    const uint8_t START_BYTE = 64;
    const size_t START_BYTE0_IDX = 0;
    const size_t START_BYTE1_IDX = 1;
    const size_t SERVICE_BYTES = 3;
    const size_t FIRST_ROS_IDX = START_BYTE1_IDX + 1;
}


#endif // SERIAL_MSG_HPP
