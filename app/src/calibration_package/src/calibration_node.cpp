#include "rclcpp/rclcpp.hpp"
#include "calibration_package/calibration_node.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationNode>();
    rclcpp::shutdown();
    return 0;
}