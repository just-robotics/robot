#include "../include/serial/msg.hpp"


Msg::Msg() {
    data_ = nullptr;
    size_ = 0;
}


Msg::Msg(size_t size, bool fill_with_zeros) : size_{size} {
    data_ = new uint8_t[size_];
    if (fill_with_zeros) {
        memset(data_, 0, size_);
    }
}


Msg::Msg(size_t size, uint8_t* data) : Msg(size, false) {
    std::copy(data, data + size_, data_);
}


Msg::Msg(size_t size, const std::initializer_list<uint8_t> array) : Msg(size, false) {
    std::copy(array.begin(), array.end(), data_);
}


Msg::~Msg() {
    delete[] data_;
}


Msg::Msg(const Msg& other) : Msg(other.size_) {
    std::copy(other.data_, other.data_ + size_, data_);
}


Msg::Msg(Msg&& other) {
    size_ = other.size_;
    data_ = other.data_;
    other.data_ = nullptr;
}

Msg& Msg::operator=(const Msg& other) {
    if (this == &other) {
        return *this;
    }

    size_ = other.size_;

    delete[] data_;
    data_ = new uint8_t[size_];
    std::copy(other.data_, other.data_ + size_, data_);

    return *this;
}


Msg& Msg::operator=(Msg&& other) {
    if (this == &other) {
        return *this;
    }

    size_ = other.size_;

    delete[] data_;
    data_ = other.data_;
    other.data_ = nullptr;

    return *this;
}


uint8_t Msg::operator[](size_t idx) {
    return data_[idx];
}


void Msg::setStartBytes() {
    if (size_ == 0) {
        return;
    }
    data_[msg_structure::START_BYTE0_IDX] = msg_structure::START_BYTE;
    data_[msg_structure::START_BYTE1_IDX] = msg_structure::START_BYTE;
}


void Msg::setChecksum(uint8_t checksum) {
    if (size_ == 0) {
        return;
    }
    data_[size_-1] = checksum;
}


void Msg::setTask(uint8_t task) {
    if (size_ == 0) {
        return;
    }
    data_[msg_structure::TASK_IDX] = task;
}


uint8_t* Msg::data() {
    return data_;
}


size_t Msg::size() {
    return size_;
}


uint8_t Msg::task() {
    if (size_ == 0) {
        throw("data is nullptr");
    }
    return data_[msg_structure::TASK_IDX];
}


void Msg::print() {
    if (size_ == 0) {
        std::cout << "size = 0" << std::endl;
        return;
    }
    std::cout << "[";
    for (size_t i = 0; i < size_; i++) {
        std::cout << static_cast<int>(data_[i]);
        if (i != size_ - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
