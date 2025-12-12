// g++ test_hiwonder_xarm_esp32.cpp hiwonder_xarm_esp32.cpp -o test_hiwonder -lboost_system -lpthread

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "hiwonder_xarm_esp32.hpp"

const std::string PORT_NAME = "/dev/ttyUSB0";  
const int BAUD_RATE = 115200; 

int main() {
    try {
        Hiwonder arm(PORT_NAME, BAUD_RATE);
        arm.ready_position();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        arm.close_hiwonder();
    } catch (const std::exception& e) {
        std::cerr << "Error : " << e.what() << std::endl;
        return 1;
    }

    return 0;
}