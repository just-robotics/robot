#include "../include/serial/msg.hpp"


Msg::Msg(size_t size) {
    size_ = size;
    msg_ = new uint8_t[size_];
    memset(msg_, 0, size_);
    msg_[0] = MsgStructure::START_BYTE;
    msg_[1] = MsgStructure::START_BYTE;
}


Msg::Msg(size_t size, const std::initializer_list<uint8_t> array) : Msg(size) {
    std::copy(array.begin(), array.end(), msg_);
}


Msg::Msg(size_t size, const uint8_t* array) : Msg(size) {
    std::copy(array, array + size_, msg_);
}


Msg::~Msg() {
    delete[] msg_;
}


Msg::Msg(const Msg& other) : Msg(other.size_) {
    std::copy(other.msg_, other.msg_ + size_, msg_);
}


Msg::Msg(Msg&& other) {
    size_ = other.size_;
    msg_ = other.msg_;
    other.msg_ = nullptr;
}

Msg& Msg::operator=(const Msg& other) {
    if (this == &other) {
        return *this;
    }

    size_ = other.size_;

    delete[] msg_;
    msg_ = new uint8_t[size_];
    std::copy(other.msg_, other.msg_ + size_, msg_);

    return *this;
}


Msg& Msg::operator=(Msg&& other) {
    if (this == &other) {
        return *this;
    }

    size_ = other.size_;

    delete[] msg_;
    msg_ = other.msg_;
    other.msg_ = nullptr;

    return *this;
}


uint8_t Msg::operator[](size_t idx) {
    return msg_[idx];
}


void Msg::set_checksum(uint8_t checksum) {
    msg_[size_-1] = checksum;
}


size_t Msg::size() {
    return size_;
}


uint8_t Msg::checksum() {
    return msg_[size_-1];
}


uint8_t* Msg::msg() {
    return msg_;
}


void Msg::print(bool is_int) {
    std::cout << "[";
    for (size_t i = 0; i < size_; i++) {
        if (is_int) {
            std::cout << static_cast<int>(msg_[i]);
        }
        else {
            std::cout << msg_[i];
        }
        if (i != size_ - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
