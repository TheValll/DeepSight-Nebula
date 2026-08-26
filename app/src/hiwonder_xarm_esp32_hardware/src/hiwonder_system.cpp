#include "hiwonder_xarm_esp32_hardware/hiwonder_system.hpp"

#include "hiwonder_xarm_rust_driver.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <unordered_set>

namespace hiwonder_xarm_esp32_hardware
{
namespace
{

constexpr std::int16_t kProtocolRawMin = 0;
constexpr std::int16_t kProtocolRawMax = 1000;

double required_double(
  const hardware_interface::ComponentInfo & joint, const std::string & parameter)
{
  const auto iterator = joint.parameters.find(parameter);
  if (iterator == joint.parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint.name + "' is missing parameter '" + parameter + "'");
  }
  return std::stod(iterator->second);
}

int required_int(
  const hardware_interface::ComponentInfo & joint, const std::string & parameter)
{
  const auto iterator = joint.parameters.find(parameter);
  if (iterator == joint.parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint.name + "' is missing parameter '" + parameter + "'");
  }
  return std::stoi(iterator->second);
}

bool required_bool(
  const hardware_interface::ComponentInfo & joint, const std::string & parameter)
{
  const auto iterator = joint.parameters.find(parameter);
  if (iterator == joint.parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint.name + "' is missing parameter '" + parameter + "'");
  }
  if (iterator->second == "true" || iterator->second == "True" || iterator->second == "1") {
    return true;
  }
  if (iterator->second == "false" || iterator->second == "False" || iterator->second == "0") {
    return false;
  }
  throw std::invalid_argument(
          "joint '" + joint.name + "' has invalid boolean parameter '" + parameter + "'");
}

bool optional_hardware_bool(
  const hardware_interface::HardwareInfo & info, const std::string & parameter,
  bool default_value)
{
  const auto iterator = info.hardware_parameters.find(parameter);
  if (iterator == info.hardware_parameters.end()) {
    return default_value;
  }
  if (iterator->second == "true" || iterator->second == "True" || iterator->second == "1") {
    return true;
  }
  if (iterator->second == "false" || iterator->second == "False" || iterator->second == "0") {
    return false;
  }
  throw std::invalid_argument("invalid boolean hardware parameter '" + parameter + "'");
}

}  // namespace

double JointCalibration::raw_to_joint(std::int16_t raw) const
{
  return ros_reference + direction * (static_cast<double>(raw) - raw_reference) * units_per_raw;
}

std::uint16_t JointCalibration::joint_to_safe_raw(double joint_position) const
{
  if (!std::isfinite(joint_position)) {
    throw std::invalid_argument("joint command is not finite");
  }
  const double raw = raw_reference +
    (joint_position - ros_reference) / (direction * units_per_raw);
  const auto rounded = static_cast<std::int32_t>(std::lround(raw));
  const auto protocol_limited = std::clamp<std::int32_t>(
    rounded, kProtocolRawMin, kProtocolRawMax);
  return static_cast<std::uint16_t>(
    std::clamp<std::int32_t>(protocol_limited, raw_min, raw_max));
}

double JointCalibration::fixed_safe_state() const
{
  return raw_to_joint(static_cast<std::int16_t>(joint_to_safe_raw(ros_reference)));
}

bool HiwonderSystem::hardware_io_permitted(bool read_only, bool joint_hardware_io_enabled)
{
  return !read_only && joint_hardware_io_enabled;
}

hardware_interface::CallbackReturn HiwonderSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    if (const auto it = info_.hardware_parameters.find("port");
      it != info_.hardware_parameters.end())
    {
      port_ = it->second;
    }
    if (const auto it = info_.hardware_parameters.find("baudrate");
      it != info_.hardware_parameters.end())
    {
      baudrate_ = std::stoi(it->second);
    }
    if (const auto it = info_.hardware_parameters.find("timeout_ms");
      it != info_.hardware_parameters.end())
    {
      timeout_ms_ = std::stoi(it->second);
    }
    if (const auto it = info_.hardware_parameters.find("startup_delay_ms");
      it != info_.hardware_parameters.end())
    {
      startup_delay_ms_ = std::stoi(it->second);
    }
    if (const auto it = info_.hardware_parameters.find("movement_duration_ms");
      it != info_.hardware_parameters.end())
    {
      movement_duration_ms_ = static_cast<std::uint16_t>(std::stoul(it->second));
    }
    read_only_ = optional_hardware_bool(info_, "read_only", false);

    if (baudrate_ <= 0 || timeout_ms_ <= 0 || startup_delay_ms_ < 0 ||
      movement_duration_ms_ == 0)
    {
      throw std::invalid_argument(
              "baudrate, timeout and movement duration must be positive; startup delay must be non-negative");
    }

    std::unordered_set<int> servo_ids;
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.empty()) {
        if (joint.name != mimic_joint_name_) {
          throw std::invalid_argument(
                  "only the gripper mimic joint may omit a command interface");
        }
        continue;
      }
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument(
                "joint '" + joint.name + "' must expose one position command interface");
      }

      JointCalibration calibration;
      calibration.joint_name = joint.name;
      const int servo_id = required_int(joint, "servo_id");
      calibration.raw_reference = required_double(joint, "raw_reference");
      calibration.ros_reference = required_double(joint, "ros_reference");
      calibration.units_per_raw = required_double(joint, "units_per_raw");
      calibration.direction = required_double(joint, "direction");
      calibration.raw_min = static_cast<std::int16_t>(required_int(joint, "raw_min"));
      calibration.raw_max = static_cast<std::int16_t>(required_int(joint, "raw_max"));
      calibration.hardware_io_enabled = required_bool(joint, "hardware_io_enabled");

      if (servo_id < 1 || servo_id > 254 || !servo_ids.insert(servo_id).second) {
        throw std::invalid_argument("servo IDs must be unique and between 1 and 254");
      }
      calibration.servo_id = static_cast<std::uint8_t>(servo_id);
      if (!std::isfinite(calibration.units_per_raw) || calibration.units_per_raw <= 0.0 ||
        (calibration.direction != -1.0 && calibration.direction != 1.0) ||
        calibration.raw_min < kProtocolRawMin || calibration.raw_max > kProtocolRawMax ||
        calibration.raw_min >= calibration.raw_max)
      {
        throw std::invalid_argument("invalid calibration for joint '" + joint.name + "'");
      }

      if (joint.name == "gripper_left_joint") {
        gripper_index_ = joints_.size();
      }
      joints_.push_back(calibration);
    }

    if (joints_.size() != 6 || gripper_index_ == std::numeric_limits<std::size_t>::max()) {
      throw std::invalid_argument("expected six commanded joints including gripper_left_joint");
    }
    previous_positions_.assign(joints_.size(), 0.0);
    last_written_raw_.assign(joints_.size(), 0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("HiwonderSystem"), "Initialization failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("HiwonderSystem"),
    "Configured six servos on %s at %d baud (position writes %s)", port_.c_str(), baudrate_,
    read_only_ ? "disabled" : "enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HiwonderSystem::connect()
{
  try {
    driver_ = std::make_unique<HiwonderRustDriver>(
      port_, baudrate_, std::chrono::milliseconds(timeout_ms_));
    if (startup_delay_ms_ > 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("HiwonderSystem"),
        "Waiting %d ms for the serial adapter/ESP32 to stabilize", startup_delay_ms_);
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("HiwonderSystem"), "Serial connection failed: %s", error.what());
    driver_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void HiwonderSystem::disconnect()
{
  driver_.reset();
}

hardware_interface::CallbackReturn HiwonderSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  return connect();
}

hardware_interface::CallbackReturn HiwonderSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HiwonderSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!driver_ && connect() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!read_all_positions(true, 0.0)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("HiwonderSystem"),
    "Hardware activated without sending an initial movement command");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HiwonderSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (driver_ && !read_only_) {
    for (const auto & joint : joints_) {
      if (!hardware_io_permitted(read_only_, joint.hardware_io_enabled)) {
        continue;
      }
      try {
        driver_->stop(joint.servo_id);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          rclcpp::get_logger("HiwonderSystem"), "Could not stop servo %u: %s",
          joint.servo_id, error.what());
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool HiwonderSystem::read_all_positions(bool initialize_commands, double period_seconds)
{
  if (!driver_) {
    return false;
  }
  try {
    double gripper_velocity = 0.0;
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const bool io_enabled = joint.hardware_io_enabled;
      const double position = io_enabled ?
        joint.raw_to_joint(driver_->get_position(joint.servo_id)) : joint.fixed_safe_state();
      const double velocity = io_enabled && period_seconds > 0.0 ?
        (position - previous_positions_[index]) / period_seconds : 0.0;
      set_state(joints_[index].joint_name + "/position", position);
      set_state(joints_[index].joint_name + "/velocity", velocity);
      if (index == gripper_index_) {
        gripper_velocity = velocity;
      }
      previous_positions_[index] = position;
      if (initialize_commands) {
        set_command(joints_[index].joint_name + "/position", position);
        last_written_raw_[index] = joint.joint_to_safe_raw(position);
      }
    }
    const double gripper_position = previous_positions_[gripper_index_];
    set_state(mimic_joint_name_ + "/position", -gripper_position);
    set_state(mimic_joint_name_ + "/velocity", -gripper_velocity);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("HiwonderSystem"), "Servo read failed: %s", error.what());
    return false;
  }
  return true;
}

hardware_interface::return_type HiwonderSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  return read_all_positions(false, period.seconds()) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HiwonderSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_) {
    return hardware_interface::return_type::ERROR;
  }
  if (read_only_) {
    return hardware_interface::return_type::OK;
  }
  try {
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!hardware_io_permitted(read_only_, joints_[index].hardware_io_enabled)) {
        continue;
      }
      const double command = get_command(joints_[index].joint_name + "/position");
      if (!std::isfinite(command)) {
        continue;
      }
      const auto raw = joints_[index].joint_to_safe_raw(command);
      if (raw != last_written_raw_[index]) {
        driver_->move(joints_[index].servo_id, raw, movement_duration_ms_);
        last_written_raw_[index] = raw;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("HiwonderSystem"), "Servo write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hiwonder_xarm_esp32_hardware

PLUGINLIB_EXPORT_CLASS(
  hiwonder_xarm_esp32_hardware::HiwonderSystem, hardware_interface::SystemInterface)
