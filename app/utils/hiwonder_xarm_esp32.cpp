#include "hiwonder_xarm_esp32.hpp"
#include <iostream>
#include <thread>
#include <chrono> 
#include <vector>

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

void Hiwonder::close_hiwonder() {
    if (serial_.is_open()) {
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
    } catch (const system::system_error& e) {
        std::cerr << "Error can't open the port : " << port_name << " : " << e.what() << std::endl;
        throw; 
    }
}

void Hiwonder::ready_position() {
    std::vector<std::string> cmds = {
        "bus_servo.run(1,500,10)",
        "bus_servo.run(2,500,10)",
        "bus_servo.run(3,200,10)",
        "bus_servo.run(4,750,10)",
        "bus_servo.run(5,500,10)",
        "bus_servo.run(6,500,10)"
    };

    for(const auto& cmd : cmds){
        send_command(cmd + "\r\n");
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    get_servos_positions(); 
}

void Hiwonder::get_servos_positions() {
    std::vector<std::string> cmds = {
        "bus_servo.get_position(1)",
        "bus_servo.get_position(2)",
        "bus_servo.get_position(3)",
        "bus_servo.get_position(4)",
        "bus_servo.get_position(5)",
        "bus_servo.get_position(6)"
    };

    std::vector<std::string> responses;

    for(const auto& cmd : cmds) {
        send_command(cmd + "\r\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        try {
            boost::asio::streambuf buf;
            boost::asio::read_until(serial_, buf, "\n");
            
            std::istream is(&buf);
            std::string line;
            std::getline(is, line);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            responses.push_back(line);
        }
        catch (boost::system::system_error& e) {
            std::cerr << "Timeout or error with the command reading " << cmd << ": " << e.what() << std::endl;
            responses.push_back("Error");
        }
    }

    std::cout << "Response :" << std::endl;
    int servo_id = 1;
    for(const auto& r : responses) {
        std::cout << "  Servo " << servo_id << " : " << r << std::endl;
        servo_id++;
    }
}