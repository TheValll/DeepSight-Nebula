#ifndef HIWONDER_XARM_ESP32_HARDWARE__HIWONDER_SYSTEM_HPP_
#define HIWONDER_XARM_ESP32_HARDWARE__HIWONDER_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

class HiwonderRustDriver;

namespace hiwonder_xarm_esp32_hardware
{

struct JointCalibration
{
  std::string joint_name;
  std::uint8_t servo_id{0};
  double raw_reference{0.0};
  double ros_reference{0.0};
  double units_per_raw{0.0};
  double direction{1.0};
  std::int16_t raw_min{0};
  std::int16_t raw_max{1000};
  bool hardware_io_enabled{true};

  double raw_to_joint(std::int16_t raw) const;
  std::uint16_t joint_to_safe_raw(double joint_position) const;
  double fixed_safe_state() const;
};

class HiwonderSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Kept public to unit-test the no-output policy without instantiating a serial driver.
  static bool hardware_io_permitted(bool read_only, bool joint_hardware_io_enabled);

private:
  hardware_interface::CallbackReturn connect();
  void disconnect();
  bool read_all_positions(bool initialize_commands, double period_seconds);

  std::unique_ptr<HiwonderRustDriver> driver_;
  std::vector<JointCalibration> joints_;
  std::vector<double> previous_positions_;
  std::vector<std::uint16_t> last_written_raw_;
  std::string port_{"/dev/ttyUSB0"};
  int baudrate_{115200};
  int timeout_ms_{50};
  int startup_delay_ms_{2000};
  std::uint16_t movement_duration_ms_{40};
  bool read_only_{false};
  std::string mimic_joint_name_{"gripper_right_joint"};
  std::size_t gripper_index_{std::numeric_limits<std::size_t>::max()};
};

}  // namespace hiwonder_xarm_esp32_hardware

#endif  // HIWONDER_XARM_ESP32_HARDWARE__HIWONDER_SYSTEM_HPP_
