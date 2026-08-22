#include "hiwonder_xarm_esp32.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace boost; 

Hiwonder::Hiwonder(const std::string& port, int baudrate) 
    : port_name(port), baudrate(baudrate), serial_(io) 
{
    try {
        open_hiwonder();
    }
    catch(const std::exception& e) {
        std::cerr << "Error configuring serial port: " << e.what() << '\n';
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
}

Hiwonder::~Hiwonder() {
    close_hiwonder();
}

void Hiwonder::setPort(const std::string& port) {
    this->port_name = port;
}

void Hiwonder::setBaudrate(int baudrate) {
    this->baudrate = baudrate;
}

std::string Hiwonder::getPort() const {
    return port_name;
}

int Hiwonder::getBaudrate() const {
    return baudrate;
}

void Hiwonder::send_command(const std::string& command) {
    if (!serial_.is_open()) {
        std::cerr << "Error serial port not open." << std::endl;
        return;
    }

    system::error_code ec;
    asio::write(serial_, asio::buffer(command), ec);

    if (ec) {
        std::cerr << "Error writing to serial port: " << ec.message() << std::endl;
    }
}

std::string Hiwonder::query(const std::string& command, const std::string& expect_prefix) {
    if (!serial_.is_open()) {
        std::cerr << "Error serial port not open." << std::endl;
        return "";
    }

    int fd = serial_.native_handle();
    
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

    termios tio;
    tcgetattr(fd, &tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &tio);

    tcflush(fd, TCIFLUSH);

    system::error_code ec;
    asio::write(serial_, asio::buffer(command), ec);
    if (ec) {
        std::cerr << "Error writing to serial port: " << ec.message() << std::endl;
        return "";
    }

    std::string line;
    char c;
    while (::read(fd, &c, 1) == 1) {
        if (c != '\n') {
            line += c;
            continue;
        }
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.rfind(expect_prefix, 0) == 0) {
            return line;
        }
        line.clear();
    }
    return "";  // timeout
}

void Hiwonder::close_hiwonder() {
    if (serial_.is_open()) {
        default_position();
        serial_.close();
        std::cout << "Serial port closed" << std::endl;
    }
}

void Hiwonder::open_hiwonder() {
    if (serial_.is_open()) {
        serial_.close();
    }

    try {
        serial_.open(port_name);       
        
        serial_.set_option(asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(asio::serial_port_base::character_size(8));
        serial_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        std::cout << "Connection opened : " << port_name << " a " << baudrate << " bauds." << std::endl;
        default_position();
    } catch (const system::system_error& e) {
        std::cerr << "Error can't open the port : " << port_name << " : " << e.what() << std::endl;
        throw; 
    }
}

void Hiwonder::ready_position() {
    std::vector<std::string> cmds = {
        "bus_servo.run(1,500,1000)",
        "bus_servo.run(2,500,1000)",
        "bus_servo.run(3,200,1000)",
        "bus_servo.run(4,750,1000)",
        "bus_servo.run(5,500,1000)",
        "bus_servo.run(6,500,1000)"
    };

    for(const auto& cmd : cmds){
        send_command(cmd + "\r\n");
    }
}

void Hiwonder::default_position() {
    std::vector<std::string> cmds = {
        "bus_servo.run(1,500,1000)",
        "bus_servo.run(2,500,1000)",
        "bus_servo.run(3,500,1000)",
        "bus_servo.run(4,500,1000)",
        "bus_servo.run(5,500,1000)",
        "bus_servo.run(6,500,1000)"
    };

    for(const auto& cmd : cmds){
        send_command(cmd + "\r\n");
    }
}