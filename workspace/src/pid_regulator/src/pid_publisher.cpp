#include <cstdint>
#include <iostream>


#define KP_IDX         2
#define KD_IDX         6
#define KI_IDX        10

#define PID_CMD_SIZE  15


int main() {

    float kp, kd, ki;

    while (true) {
        std::cout << "Enter kp, kd, ki:" << std::endl;
        std::cin >> kp >> kd >> ki;
        std::cout << "kp = " << kp << "; kd = " << kd << "; ki = " << ki << ";" << std::endl;
    }

    return 0;
}
