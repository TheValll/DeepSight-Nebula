// g++ test_hiwonder_xarm_esp32.cpp hiwonder_xarm_esp32.cpp -o test_hiwonder -lboost_system -lpthread

#include <iostream>
#include <string>
#include "hiwonder_xarm_esp32.hpp"

const std::string PORT_NAME = "COM8";  
const int BAUD_RATE = 115200; 

int main() {
    try {
        Hiwonder arm(PORT_NAME, BAUD_RATE);
        arm.ready_position();
    } catch (const std::exception& e) {
        std::cerr << "Error : " << e.what() << std::endl;
        return 1;
    }

    return 0;
}