#include <iostream>

#include "../include/serial/connect.hpp"

int a;

int main(int, char**){
    Connect::init();

    if (!Connect::setConnection()) {
        return 1;
    };

    while (true) {

        Msg on(6, {64, 64, 6, 0, 0, 0});
        Msg off(6, {64, 64, 7, 0, 0, 0});

        std::cin >> a;
        if (a == 5) {
            Connect::sendCommand(&on);
        }
        if (a == 6) {
            Connect::sendCommand(&off);
        }
        
        Msg feedback = Connect::receiveMessage();
        feedback.print();
    }
}
