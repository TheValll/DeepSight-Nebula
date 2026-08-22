#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include "hiwonder_xarm_esp32.hpp"

const std::string PORT_NAME = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;
const int ITERATIONS = 1000;
const int WARMUP = 20;
const int PAUSE_MS = 10;
const std::string CSV_PATH = "benchmarks/micropython_latency.csv";

int main() {
    try {
        Hiwonder arm(PORT_NAME, BAUD_RATE);

        if (arm.query("print('SYNC')\r\n", "SYNC") != "SYNC") {
            std::cerr << "Error: no answer from the MicroPython REPL" << std::endl;
            return 1;
        }

        std::string pos_line = arm.query("print('P', bus_servo.get_position(1))\r\n", "P ");
        std::string current_pos = pos_line.substr(2);
        if (current_pos.empty() || current_pos == "False") {
            std::cerr << "Error: could not read the position of servo 1" << std::endl;
            return 1;
        }
        std::cout << "Servo 1 position: " << current_pos << std::endl;

        const std::string scenarios[3][3] = {
            {"floor", "print(1)\r\n", "1"},
            {"read", "t0=time.ticks_us(); p=bus_servo.get_position(1); print('R', p, time.ticks_diff(time.ticks_us(), t0))\r\n", "R "},
            {"write", "bus_servo.run(1, " + current_pos + ", 1000); print('W')\r\n", "W"},
        };

        std::filesystem::create_directories("benchmarks");
        std::ofstream csv(CSV_PATH);
        csv << "scenario,iteration,roundtrip_us,response\n";

        for (const auto& s : scenarios) {
            const std::string &name = s[0], &command = s[1], &expect = s[2];
            std::cout << "[" << name << "] running..." << std::endl;

            for (int i = -WARMUP; i < ITERATIONS; ++i) {
                auto t0 = std::chrono::steady_clock::now();
                std::string response = arm.query(command, expect);
                auto t1 = std::chrono::steady_clock::now();
                long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

                if (i >= 0) { 
                    csv << name << "," << i + 1 << "," << us << "," << response << "\n";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_MS));
            }
        }

        csv.close();
        std::cout << "Done: " << CSV_PATH << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error : " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
