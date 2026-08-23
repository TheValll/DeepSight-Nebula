#include "hiwonder_xarm_rust_driver.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    const std::string port = argc > 1
                                 ? argv[1]
                                 : "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
    const bool safe_write_test = argc > 2 &&
                                 std::string(argv[2]) == "--safe-write";

    try {
        HiwonderRustDriver arm(port, 115200, std::chrono::milliseconds(50));
        // Opening some CH340 adapters can reset the ESP32 through DTR/RTS.
        std::this_thread::sleep_for(std::chrono::seconds(2));

        int successful_servos = 0;
        for (std::uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
            std::cout << "Servo " << static_cast<unsigned>(servo_id) << ": ";
            try {
                const auto reported_id = arm.get_id(servo_id);
                const auto position = arm.get_position(servo_id);
                const auto voltage_mv = arm.get_vin(servo_id);
                const auto offset = arm.get_offset(servo_id);
                const auto move_time = arm.read_move_time(servo_id);

                std::cout << "id=" << static_cast<unsigned>(reported_id)
                          << ", position=" << position
                          << ", voltage=" << voltage_mv << " mV"
                          << ", offset=" << static_cast<int>(offset)
                          << ", target=" << move_time.position
                          << ", duration=" << move_time.time_ms << " ms\n";
                ++successful_servos;
            } catch (const std::exception& error) {
                std::cout << "ERROR: " << error.what() << '\n';
            }
        }

        std::cout << successful_servos << "/6 servos read successfully\n";
        if (successful_servos != 6) {
            return 2;
        }

        if (safe_write_test) {
            const auto initial_position = arm.get_position(1);
            if (initial_position < 0 || initial_position > 1000) {
                throw std::runtime_error(
                    "servo 1 position is outside the safe command range");
            }

            arm.move(1, static_cast<std::uint16_t>(initial_position), 250);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            arm.stop(1);
            const auto final_position = arm.get_position(1);
            std::cout << "Safe write test passed: servo 1 remained at "
                      << final_position << " (initial " << initial_position
                      << ")\n";
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Unable to test " << port << ": " << error.what() << '\n';
        return 1;
    }
}
