#ifndef SERIAL_MSG_HPP
#define SERIAL_MSG_HPP


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>


class Msg {
private:
    inline static uint8_t START_BYTE;

    size_t size_;
    uint8_t* msg_;

public:
    static void init();

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


#endif // SERIAL_MSG_HPP
