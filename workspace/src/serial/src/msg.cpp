#include "../include/serial/msg.hpp"


Msg::Msg() {
    size_ = 0;
    task_= Tasks::PING;
}


Msg::Msg(size_t size) {
    size_ = size;
    task_ = Tasks::PING;
    msg_ = new uint8_t[size_];
    memset(msg_, 0, size_);
    msg_[MsgStructure::START_BYTE0_IDX] = MsgStructure::START_BYTE;
    msg_[MsgStructure::START_BYTE1_IDX] = MsgStructure::START_BYTE;
    msg_[MsgStructure::TASK_IDX] = task_;
}


Msg::Msg(size_t size, const std::initializer_list<uint8_t> array) : Msg(size) {
    std::copy(array.begin(), array.end(), msg_);
}


Msg::Msg(size_t size, const uint8_t* array) : Msg(size) {
    std::copy(array, array + size_, msg_);
}


Msg::~Msg() {
    if (size_ != 0) {
        delete[] msg_;
    }
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


void Msg::set_task(uint8_t task) {
    task_ = task;
    msg_[MsgStructure::TASK_IDX] = task_;
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


uint8_t Msg::task() {
    return task_;
}


void Msg::print() {
    if (size_ == 0) {
        std::cout << "size = 0" << std::endl;
        return;
    }
    std::cout << "[";
    for (size_t i = 0; i < size_; i++) {
        std::cout << static_cast<int>(msg_[i]);
        if (i != size_ - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
