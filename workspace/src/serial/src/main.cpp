#include <iostream>

#include "../include/serial/connect.hpp"

int a;

int main(int, char**){
    if (!Connect::setConnection()) {
        return 1;
    };

    while (true) {
        
    }
}
