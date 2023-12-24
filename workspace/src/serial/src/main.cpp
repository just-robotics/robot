#include <iostream>

#include "../include/serial/connect.hpp"


#define POSE_IDX       2
#define TARGET_IDX    11
#define U_IDX         20

#define POSE_SIZE      9
#define TARGET_SIZE    9
#define U_SIZE         9

#define POSE_MSG_SIZE 30


float kp = 1.0;
float kd = 0.0;
float ki = 0.0;

int64_t pose = 0;
int64_t target = 0;
float u = 0;


uint8_t* int64_to_uint8arr(int64_t number) {
    uint8_t byte = 0x000000FF;
    uint8_t* data = new uint8_t[9];

    data[8] = number < 0 ? 0 : 1;
    uint64_t u_number = std::abs(number);

    data[0] = u_number & byte;
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        data[i] = u_number & byte;
    }

    return data;
}


int64_t uint8arr_to_int64(uint8_t* data) {
    int64_t number = data[7];

    for (int i = 6; i >= 0; i--) {
        number <<= 8;
        number = number | data[i];
    }

    number = data[8] == 1 ? number : -number;
    
    return number;
}


int main(int, char**){
    if (!Connect::setConnection()) {
        return 1;
    };

    while (true) {
        Msg msg = Connect::receiveMessage(POSE_MSG_SIZE);
        if (Connect::checkFeedback()) {
            pose = uint8arr_to_int64(msg.msg() + POSE_IDX);
            target = uint8arr_to_int64(msg.msg() + TARGET_IDX);
            u = uint8arr_to_int64(msg.msg() + U_IDX);
            std::cout << pose << " " << target << " " << u << std::endl;
            // msg.print();
            // std::cout << "there\n";
        }
        Connect::delay(100);
    }
}
