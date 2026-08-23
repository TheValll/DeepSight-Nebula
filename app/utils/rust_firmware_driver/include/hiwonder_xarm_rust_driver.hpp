#ifndef HIWONDER_XARM_RUST_DRIVER_HPP
#define HIWONDER_XARM_RUST_DRIVER_HPP

#include <boost/asio.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class HiwonderRustDriver {
public:
    struct MoveTime {
        std::uint16_t position;
        std::uint16_t time_ms;
    };

    explicit HiwonderRustDriver(
        const std::string& port,
        int baudrate = 115200,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(50));
    ~HiwonderRustDriver();

    HiwonderRustDriver(const HiwonderRustDriver&) = delete;
    HiwonderRustDriver& operator=(const HiwonderRustDriver&) = delete;

    void setPort(const std::string& port);
    void setBaudrate(int baudrate);
    void setTimeout(std::chrono::milliseconds timeout);

    std::string getPort() const;
    int getBaudrate() const;
    std::chrono::milliseconds getTimeout() const;
    bool isOpen() const;

    void open_hiwonder();
    void close_hiwonder();

    void move(std::uint8_t servo_id, std::uint16_t position,
              std::uint16_t time_ms);
    MoveTime read_move_time(std::uint8_t servo_id);
    void stop(std::uint8_t servo_id);

    void set_id(std::uint8_t servo_id, std::uint8_t new_id);
    std::uint8_t get_id(std::uint8_t servo_id);

    void adjust_offset(std::uint8_t servo_id, std::int8_t offset);
    void save_offset(std::uint8_t servo_id);
    std::int8_t get_offset(std::uint8_t servo_id);

    std::uint16_t get_vin(std::uint8_t servo_id);
    std::int16_t get_position(std::uint8_t servo_id);

    void set_mode(std::uint8_t servo_id, std::uint8_t mode,
                  std::int16_t speed = 0);
    void load(std::uint8_t servo_id);
    void unload(std::uint8_t servo_id);

    void ready_position();
    void default_position();

    static std::vector<std::uint8_t> build_frame(
        std::uint8_t servo_id, std::uint8_t command,
        const std::vector<std::uint8_t>& parameters = {});
    static bool is_valid_frame(const std::vector<std::uint8_t>& frame);

private:
    static constexpr std::uint8_t SERVO_MOVE_TIME_WRITE = 0x01;
    static constexpr std::uint8_t SERVO_MOVE_TIME_READ = 0x02;
    static constexpr std::uint8_t SERVO_MOVE_STOP = 0x0C;
    static constexpr std::uint8_t SERVO_ID_WRITE = 0x0D;
    static constexpr std::uint8_t SERVO_ID_READ = 0x0E;
    static constexpr std::uint8_t SERVO_ANGLE_OFFSET_ADJUST = 0x11;
    static constexpr std::uint8_t SERVO_ANGLE_OFFSET_WRITE = 0x12;
    static constexpr std::uint8_t SERVO_ANGLE_OFFSET_READ = 0x13;
    static constexpr std::uint8_t SERVO_VIN_READ = 0x1B;
    static constexpr std::uint8_t SERVO_POS_READ = 0x1C;
    static constexpr std::uint8_t SERVO_OR_MOTOR_MODE_WRITE = 0x1D;
    static constexpr std::uint8_t SERVO_LOAD_OR_UNLOAD_WRITE = 0x1F;

    void send_command(std::uint8_t servo_id, std::uint8_t command,
                      const std::vector<std::uint8_t>& parameters = {});
    std::vector<std::uint8_t> query(
        std::uint8_t servo_id, std::uint8_t command,
        std::size_t expected_parameter_count);
    std::vector<std::uint8_t> read_frame(
        std::chrono::steady_clock::time_point deadline);
    void ensure_open() const;
    void flush_input();
    static void validate_servo_id(std::uint8_t servo_id);

    std::string port_name;
    int baudrate;
    std::chrono::milliseconds timeout;
    boost::asio::io_context io;
    boost::asio::serial_port serial_;
    mutable std::mutex transaction_mutex_;
};

#endif
