#ifndef HIWONDER_XARM_ESP32_HPP
#define HIWONDER_XARM_ESP32_HPP

#include <iostream>
#include <string>
#include <vector>
#include <boost/asio.hpp>

class Hiwonder {
public:
    Hiwonder(const std::string& port, int baudrate);
    ~Hiwonder();

    void setPort(const std::string& port);
    void setBaudrate(int baudrate);

    std::string getPort() const;
    int getBaudrate() const;

    void send_command(const std::string& command);
    std::string query(const std::string& command, const std::string& expect_prefix);

    void close_hiwonder();
    void open_hiwonder();
    
    void ready_position();
    void default_position();
    // void get_servos_positions();

private:
    std::string port_name;
    int baudrate;

    boost::asio::io_context io;
    boost::asio::serial_port serial_;
};

#endif