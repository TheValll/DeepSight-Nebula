#include "hiwonder_xarm_rust_driver.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
    const std::vector<std::uint8_t> position_read{
        0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF};
    assert(HiwonderRustDriver::build_frame(1, 0x1C) == position_read);
    assert(HiwonderRustDriver::is_valid_frame(position_read));

    const std::vector<std::uint8_t> move{
        0x55, 0x55, 0x01, 0x07, 0x01,
        0xF4, 0x01, 0xE8, 0x03, 0x16};
    assert(HiwonderRustDriver::build_frame(
               1, 0x01, {0xF4, 0x01, 0xE8, 0x03}) == move);
    assert(HiwonderRustDriver::is_valid_frame(move));

    auto corrupted = move;
    corrupted.back() ^= 0x01;
    assert(!HiwonderRustDriver::is_valid_frame(corrupted));

    std::cout << "Protocol frame tests passed" << std::endl;
    return 0;
}
