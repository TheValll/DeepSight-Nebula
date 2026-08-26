#include "hiwonder_xarm_rust_driver.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char * kDefaultPort = "/dev/ttyUSB0";
constexpr int kDefaultBaudrate = 115200;
constexpr int kDefaultReadCount = 1;
constexpr int kDefaultIntervalMs = 250;

struct Options {
  std::string command;
  std::string port{kDefaultPort};
  int baudrate{kDefaultBaudrate};
  int id{0};
  int count{kDefaultReadCount};
  int interval_ms{kDefaultIntervalMs};
  int target{-1};
  int duration_ms{-1};
  bool execute{false};
  bool torque_on{false};
  bool torque_off{false};
  bool count_specified{false};
  bool interval_specified{false};
};

void print_usage(std::ostream & stream)
{
  stream << "Usage:\n"
         << "  servo_calibration_cli read --id ID [--count N] [--interval-ms MS]"
            " [--port DEVICE] [--baudrate RATE]\n"
         << "  servo_calibration_cli move --id ID --target RAW --duration-ms MS"
            " --execute [--port DEVICE] [--baudrate RATE]\n\n"
         << "  servo_calibration_cli torque --id ID (--off|--on) --execute"
            " [--port DEVICE] [--baudrate RATE]\n\n"
         << "read only queries the current raw position. move and torque send no command unless"
            " --execute is present.\n"
         << "ID: 1..254; RAW: 0..1000; duration: 1..60000 ms.\n";
}

std::optional<int> parse_integer(const std::string & text)
{
  std::size_t parsed = 0;
  try {
    const long long value = std::stoll(text, &parsed, 10);
    if (parsed != text.size() || value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max()) {
      return std::nullopt;
    }
    return static_cast<int>(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool assign_value(const std::string & name, const std::string & value, int & target,
                  std::string & error)
{
  const auto parsed = parse_integer(value);
  if (!parsed) {
    error = name + " must be an integer";
    return false;
  }
  target = *parsed;
  return true;
}

bool parse_options(int argc, char ** argv, Options & options, std::string & error)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    options.command = "help";
    return true;
  }
  if (argc < 2) {
    error = "a subcommand is required";
    return false;
  }
  options.command = argv[1];
  if (options.command != "read" && options.command != "move" &&
      options.command != "torque") {
    error = "subcommand must be read, move, or torque";
    return false;
  }

  for (int index = 2; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--execute") {
      options.execute = true;
      continue;
    }
    if (argument == "--on") {
      options.torque_on = true;
      continue;
    }
    if (argument == "--off") {
      options.torque_off = true;
      continue;
    }
    if (argument == "--help") {
      options.command = "help";
      return true;
    }
    if (index + 1 == argc) {
      error = "missing value after " + argument;
      return false;
    }
    const std::string value = argv[++index];
    if (argument == "--id") {
      if (!assign_value(argument, value, options.id, error)) return false;
    } else if (argument == "--count") {
      if (!assign_value(argument, value, options.count, error)) return false;
      options.count_specified = true;
    } else if (argument == "--interval-ms") {
      if (!assign_value(argument, value, options.interval_ms, error)) return false;
      options.interval_specified = true;
    } else if (argument == "--target") {
      if (!assign_value(argument, value, options.target, error)) return false;
    } else if (argument == "--duration-ms") {
      if (!assign_value(argument, value, options.duration_ms, error)) return false;
    } else if (argument == "--baudrate") {
      if (!assign_value(argument, value, options.baudrate, error)) return false;
    } else if (argument == "--port") {
      options.port = value;
    } else {
      error = "unknown option " + argument;
      return false;
    }
  }

  if (options.id < 1 || options.id > 254) {
    error = "--id must be between 1 and 254";
  } else if (options.baudrate <= 0) {
    error = "--baudrate must be positive";
  } else if (options.command == "read" && options.target != -1) {
    error = "--target is only valid with move";
  } else if (options.command == "read" && options.duration_ms != -1) {
    error = "--duration-ms is only valid with move";
  } else if (options.command == "read" && options.execute) {
    error = "--execute is only valid with move or torque";
  } else if (options.command == "read" && (options.torque_on || options.torque_off)) {
    error = "--on and --off are only valid with torque";
  } else if (options.command == "read" && (options.count < 1 || options.count > 100)) {
    error = "--count must be between 1 and 100";
  } else if (options.command == "read" && (options.interval_ms < 0 || options.interval_ms > 60000)) {
    error = "--interval-ms must be between 0 and 60000";
  } else if (options.command == "move" && (options.target < 0 || options.target > 1000)) {
    error = "move requires --target between 0 and 1000";
  } else if (options.command == "move" && (options.duration_ms < 1 || options.duration_ms > 60000)) {
    error = "move requires --duration-ms between 1 and 60000";
  } else if (options.command == "move" && !options.execute) {
    error = "refusing to move without the explicit --execute confirmation";
  } else if (options.command == "move" && (options.torque_on || options.torque_off)) {
    error = "--on and --off are only valid with torque";
  } else if (options.command == "torque" && options.target != -1) {
    error = "--target is only valid with move";
  } else if (options.command == "torque" && options.duration_ms != -1) {
    error = "--duration-ms is only valid with move";
  } else if (options.command == "torque" && options.count_specified) {
    error = "--count is only valid with read";
  } else if (options.command == "torque" && options.interval_specified) {
    error = "--interval-ms is only valid with read";
  } else if (options.command == "torque" && options.torque_on == options.torque_off) {
    error = "torque requires exactly one of --on or --off";
  } else if (options.command == "torque" && !options.execute) {
    error = "refusing to change torque without the explicit --execute confirmation";
  }
  return error.empty();
}

int run_read(const Options & options)
{
  HiwonderRustDriver driver(options.port, options.baudrate);
  for (int sample = 0; sample < options.count; ++sample) {
    const auto raw = driver.get_position(static_cast<std::uint8_t>(options.id));
    std::cout << "sample=" << sample + 1 << " id=" << options.id
              << " raw=" << raw << '\n';
    if (sample + 1 < options.count && options.interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
    }
  }
  return 0;
}

int run_move(const Options & options)
{
  HiwonderRustDriver driver(options.port, options.baudrate);
  const auto id = static_cast<std::uint8_t>(options.id);
  driver.move(id, static_cast<std::uint16_t>(options.target),
              static_cast<std::uint16_t>(options.duration_ms));
  std::cout << "move sent: id=" << options.id << " target=" << options.target
            << " duration_ms=" << options.duration_ms << '\n';
  std::this_thread::sleep_for(std::chrono::milliseconds(options.duration_ms));
  driver.stop(id);
  const auto raw = driver.get_position(id);
  std::cout << "stopped: id=" << options.id << " final_raw=" << raw << '\n';
  return 0;
}

int run_torque(const Options & options)
{
  HiwonderRustDriver driver(options.port, options.baudrate);
  const auto id = static_cast<std::uint8_t>(options.id);
  if (options.torque_off) {
    driver.unload(id);
    std::cout << "torque disabled: id=" << options.id << '\n';
  } else {
    driver.load(id);
    std::cout << "torque enabled: id=" << options.id << '\n';
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  std::string error;
  if (!parse_options(argc, argv, options, error)) {
    std::cerr << "Error: " << error << "\n\n";
    print_usage(std::cerr);
    return 2;
  }
  if (options.command == "help") {
    print_usage(std::cout);
    return 0;
  }
  try {
    if (options.command == "read") return run_read(options);
    if (options.command == "move") return run_move(options);
    return run_torque(options);
  } catch (const std::exception & exception) {
    std::cerr << "Servo operation failed: " << exception.what() << '\n';
    return 1;
  }
}
