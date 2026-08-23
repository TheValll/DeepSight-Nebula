#include "hiwonder_xarm_rust_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int ITERATIONS = 1000;
constexpr int WARMUP = 20;
constexpr int PAUSE_MS = 10;
constexpr std::uint8_t SERVO_ID = 1;

struct Sample {
    long long roundtrip_us;
    bool success;
    std::string response;
};

std::string csv_escape(const std::string& value) {
    std::string escaped = "\"";
    for (const char character : value) {
        if (character == '"') {
            escaped += "\"\"";
        } else {
            escaped += character;
        }
    }
    escaped += '"';
    return escaped;
}

void print_summary(const std::string& name,
                   const std::vector<Sample>& samples) {
    std::vector<long long> successful;
    for (const auto& sample : samples) {
        if (sample.success) {
            successful.push_back(sample.roundtrip_us);
        }
    }

    std::cout << '[' << name << "] " << successful.size() << '/'
              << samples.size() << " successful";
    if (successful.empty()) {
        std::cout << '\n';
        return;
    }

    std::sort(successful.begin(), successful.end());
    const double mean = static_cast<double>(
                            std::accumulate(successful.begin(), successful.end(),
                                            0LL)) /
                        static_cast<double>(successful.size());
    const auto median = successful[successful.size() / 2];
    const auto p95_index = static_cast<std::size_t>(
        std::ceil(static_cast<double>(successful.size()) * 0.95)) - 1U;

    std::cout << ", mean=" << std::fixed << std::setprecision(1) << mean
              << " us, median=" << median << " us, p95="
              << successful[p95_index] << " us\n";
}

template <typename Operation>
std::vector<Sample> run_scenario(const std::string& name,
                                 Operation operation,
                                 std::ofstream& csv) {
    std::cout << '[' << name << "] running..." << std::endl;
    std::vector<Sample> samples;
    samples.reserve(ITERATIONS);

    for (int iteration = -WARMUP; iteration < ITERATIONS; ++iteration) {
        Sample sample{0, false, {}};
        const auto start = std::chrono::steady_clock::now();
        try {
            sample.response = operation();
            sample.success = true;
        } catch (const std::exception& error) {
            sample.response = error.what();
        }
        const auto end = std::chrono::steady_clock::now();
        sample.roundtrip_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count();

        if (iteration >= 0) {
            samples.push_back(sample);
            csv << name << ',' << iteration + 1 << ',' << sample.roundtrip_us
                << ',' << (sample.success ? 1 : 0) << ','
                << csv_escape(sample.response) << '\n';
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_MS));
    }

    print_summary(name, samples);
    return samples;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string port = argc > 1
                                 ? argv[1]
                                 : "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";

    try {
        HiwonderRustDriver arm(port, 115200, std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::seconds(2));

        const auto current_position = arm.get_position(SERVO_ID);
        if (current_position < 0 || current_position > 1000) {
            throw std::runtime_error("servo 1 position is outside 0..1000");
        }
        std::cout << "Servo 1 position: " << current_position << '\n';

        char timestamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%Hh%M",
                      std::localtime(&now));
        std::filesystem::create_directories("benchmarks");
        const std::string csv_path =
            "benchmarks/" + std::string(timestamp) + "_rust_latency.csv";
        std::ofstream csv(csv_path);
        if (!csv) {
            throw std::runtime_error("cannot create " + csv_path);
        }
        csv << "scenario,iteration,roundtrip_us,success,response\n";

        run_scenario("host_floor", [] {
            const auto frame = HiwonderRustDriver::build_frame(1, 0x1C);
            return std::to_string(frame.size());
        }, csv);

        run_scenario("read", [&arm] {
            return std::to_string(arm.get_position(SERVO_ID));
        }, csv);

        run_scenario("write", [&arm, current_position] {
            arm.move(SERVO_ID,
                     static_cast<std::uint16_t>(current_position), 1000);
            return std::string("W");
        }, csv);

        std::cout << "Done: " << csv_path << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
