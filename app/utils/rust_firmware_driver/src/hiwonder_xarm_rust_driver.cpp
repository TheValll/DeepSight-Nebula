#include "hiwonder_xarm_rust_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace {

std::uint16_t read_u16_le(const std::vector<std::uint8_t>& data,
                          std::size_t offset) {
    return static_cast<std::uint16_t>(data.at(offset)) |
           (static_cast<std::uint16_t>(data.at(offset + 1)) << 8U);
}

std::vector<std::uint8_t> encode_u16(std::uint16_t value) {
    return {static_cast<std::uint8_t>(value & 0xFFU),
            static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
}

}  // namespace

HiwonderRustDriver::HiwonderRustDriver(
    const std::string& port, int baudrate,
    std::chrono::milliseconds timeout)
    : port_name(port),
      baudrate(baudrate),
      timeout(timeout),
      serial_(io) {
    open_hiwonder();
}

HiwonderRustDriver::~HiwonderRustDriver() {
    try {
        close_hiwonder();
    } catch (...) {
    }
}

void HiwonderRustDriver::setPort(const std::string& port) {
    if (isOpen()) {
        throw std::logic_error("close the serial port before changing it");
    }
    port_name = port;
}

void HiwonderRustDriver::setBaudrate(int value) {
    if (isOpen()) {
        throw std::logic_error("close the serial port before changing baudrate");
    }
    if (value <= 0) {
        throw std::invalid_argument("baudrate must be positive");
    }
    baudrate = value;
}

void HiwonderRustDriver::setTimeout(std::chrono::milliseconds value) {
    if (value.count() <= 0) {
        throw std::invalid_argument("timeout must be positive");
    }
    timeout = value;
}

std::string HiwonderRustDriver::getPort() const { return port_name; }
int HiwonderRustDriver::getBaudrate() const { return baudrate; }
std::chrono::milliseconds HiwonderRustDriver::getTimeout() const {
    return timeout;
}
bool HiwonderRustDriver::isOpen() const { return serial_.is_open(); }

void HiwonderRustDriver::open_hiwonder() {
    std::lock_guard<std::mutex> lock(transaction_mutex_);
    if (serial_.is_open()) {
        return;
    }
    if (baudrate <= 0) {
        throw std::invalid_argument("baudrate must be positive");
    }
    if (timeout.count() <= 0) {
        throw std::invalid_argument("timeout must be positive");
    }

    serial_.open(port_name);
    try {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
        flush_input();
    } catch (...) {
        boost::system::error_code ignored;
        serial_.close(ignored);
        throw;
    }
}

void HiwonderRustDriver::close_hiwonder() {
    std::lock_guard<std::mutex> lock(transaction_mutex_);
    if (serial_.is_open()) {
        serial_.close();
    }
}

std::vector<std::uint8_t> HiwonderRustDriver::build_frame(
    std::uint8_t servo_id, std::uint8_t command,
    const std::vector<std::uint8_t>& parameters) {
    validate_servo_id(servo_id);
    if (parameters.size() > 252) {
        throw std::length_error("too many servo command parameters");
    }

    const auto length = static_cast<std::uint8_t>(parameters.size() + 3U);
    std::vector<std::uint8_t> frame{0x55, 0x55, servo_id, length, command};
    frame.insert(frame.end(), parameters.begin(), parameters.end());

    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < frame.size(); ++i) {
        sum = static_cast<std::uint8_t>(sum + frame[i]);
    }
    frame.push_back(static_cast<std::uint8_t>(~sum));
    return frame;
}

bool HiwonderRustDriver::is_valid_frame(
    const std::vector<std::uint8_t>& frame) {
    if (frame.size() < 6 || frame[0] != 0x55 || frame[1] != 0x55 ||
        static_cast<std::size_t>(frame[3]) + 3U != frame.size()) {
        return false;
    }
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < frame.size(); ++i) {
        sum = static_cast<std::uint8_t>(sum + frame[i]);
    }
    return sum == 0xFF;
}

void HiwonderRustDriver::send_command(
    std::uint8_t servo_id, std::uint8_t command,
    const std::vector<std::uint8_t>& parameters) {
    std::lock_guard<std::mutex> lock(transaction_mutex_);
    ensure_open();
    const auto frame = build_frame(servo_id, command, parameters);
    boost::asio::write(serial_, boost::asio::buffer(frame));
}

std::vector<std::uint8_t> HiwonderRustDriver::query(
    std::uint8_t servo_id, std::uint8_t command,
    std::size_t expected_parameter_count) {
    std::lock_guard<std::mutex> lock(transaction_mutex_);
    ensure_open();
    flush_input();

    const auto request = build_frame(servo_id, command);
    boost::asio::write(serial_, boost::asio::buffer(request));
    const auto response = read_frame(std::chrono::steady_clock::now() + timeout);

    if (response[2] != servo_id) {
        throw std::runtime_error("response servo ID does not match request");
    }
    if (response[4] != command) {
        throw std::runtime_error("response command does not match request");
    }
    if (response.size() != expected_parameter_count + 6U) {
        throw std::runtime_error("unexpected response parameter count");
    }
    return {response.begin() + 5, response.end() - 1};
}

std::vector<std::uint8_t> HiwonderRustDriver::read_frame(
    std::chrono::steady_clock::time_point deadline) {
    std::vector<std::uint8_t> frame;
    frame.reserve(10);
    const int fd = serial_.native_handle();

    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const int wait_ms = static_cast<int>(
            std::max<std::int64_t>(1, remaining.count()));
        pollfd descriptor{fd, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, wait_ms);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::system_error(errno, std::generic_category(), "poll");
        }
        if (ready == 0) {
            break;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            throw std::runtime_error("serial port failed while reading response");
        }

        std::uint8_t byte = 0;
        const ssize_t count = ::read(fd, &byte, 1);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            throw std::system_error(errno, std::generic_category(), "read");
        }
        if (count == 0) {
            continue;
        }

        if (frame.empty()) {
            if (byte == 0x55) {
                frame.push_back(byte);
            }
            continue;
        }
        if (frame.size() == 1) {
            if (byte == 0x55) {
                frame.push_back(byte);
            } else {
                frame.clear();
            }
            continue;
        }

        frame.push_back(byte);
        if (frame.size() == 4 && (frame[3] < 3 || frame[3] > 7)) {
            frame.clear();
            continue;
        }
        if (frame.size() >= 4 && frame.size() == frame[3] + 3U) {
            if (!is_valid_frame(frame)) {
                throw std::runtime_error("invalid servo response checksum");
            }
            return frame;
        }
    }
    throw std::runtime_error("servo response timeout");
}

void HiwonderRustDriver::ensure_open() const {
    if (!serial_.is_open()) {
        throw std::logic_error("serial port is not open");
    }
}

void HiwonderRustDriver::flush_input() {
    if (::tcflush(serial_.native_handle(), TCIFLUSH) != 0) {
        throw std::system_error(errno, std::generic_category(), "tcflush");
    }
}

void HiwonderRustDriver::validate_servo_id(std::uint8_t servo_id) {
    if (servo_id == 0 || servo_id == 0xFF) {
        throw std::invalid_argument("servo ID must be between 1 and 254");
    }
}

void HiwonderRustDriver::move(std::uint8_t servo_id,
                              std::uint16_t position,
                              std::uint16_t time_ms) {
    if (position > 1000) {
        throw std::out_of_range("servo position must be between 0 and 1000");
    }
    auto parameters = encode_u16(position);
    const auto encoded_time = encode_u16(time_ms);
    parameters.insert(parameters.end(), encoded_time.begin(), encoded_time.end());
    send_command(servo_id, SERVO_MOVE_TIME_WRITE, parameters);
}

HiwonderRustDriver::MoveTime HiwonderRustDriver::read_move_time(
    std::uint8_t servo_id) {
    const auto data = query(servo_id, SERVO_MOVE_TIME_READ, 4);
    return {read_u16_le(data, 0), read_u16_le(data, 2)};
}

void HiwonderRustDriver::stop(std::uint8_t servo_id) {
    send_command(servo_id, SERVO_MOVE_STOP);
}

void HiwonderRustDriver::set_id(std::uint8_t servo_id,
                                std::uint8_t new_id) {
    validate_servo_id(new_id);
    send_command(servo_id, SERVO_ID_WRITE, {new_id});
}

std::uint8_t HiwonderRustDriver::get_id(std::uint8_t servo_id) {
    return query(servo_id, SERVO_ID_READ, 1)[0];
}

void HiwonderRustDriver::adjust_offset(std::uint8_t servo_id,
                                       std::int8_t offset) {
    send_command(servo_id, SERVO_ANGLE_OFFSET_ADJUST,
                 {static_cast<std::uint8_t>(offset)});
}

void HiwonderRustDriver::save_offset(std::uint8_t servo_id) {
    send_command(servo_id, SERVO_ANGLE_OFFSET_WRITE);
}

std::int8_t HiwonderRustDriver::get_offset(std::uint8_t servo_id) {
    return static_cast<std::int8_t>(
        query(servo_id, SERVO_ANGLE_OFFSET_READ, 1)[0]);
}

std::uint16_t HiwonderRustDriver::get_vin(std::uint8_t servo_id) {
    const auto data = query(servo_id, SERVO_VIN_READ, 2);
    return read_u16_le(data, 0);
}

std::int16_t HiwonderRustDriver::get_position(std::uint8_t servo_id) {
    const auto data = query(servo_id, SERVO_POS_READ, 2);
    return static_cast<std::int16_t>(read_u16_le(data, 0));
}

void HiwonderRustDriver::set_mode(std::uint8_t servo_id,
                                  std::uint8_t mode,
                                  std::int16_t speed) {
    if (mode > 1) {
        throw std::invalid_argument("mode must be 0 (servo) or 1 (motor)");
    }
    const auto encoded_speed = encode_u16(static_cast<std::uint16_t>(speed));
    send_command(servo_id, SERVO_OR_MOTOR_MODE_WRITE,
                 {mode, 0x00, encoded_speed[0], encoded_speed[1]});
}

void HiwonderRustDriver::load(std::uint8_t servo_id) {
    send_command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, {1});
}

void HiwonderRustDriver::unload(std::uint8_t servo_id) {
    send_command(servo_id, SERVO_LOAD_OR_UNLOAD_WRITE, {0});
}

void HiwonderRustDriver::ready_position() {
    move(1, 500, 1000);
    move(2, 500, 1000);
    move(3, 200, 1000);
    move(4, 750, 1000);
    move(5, 500, 1000);
    move(6, 500, 1000);
}

void HiwonderRustDriver::default_position() {
    for (std::uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        move(servo_id, 500, 1000);
    }
}
