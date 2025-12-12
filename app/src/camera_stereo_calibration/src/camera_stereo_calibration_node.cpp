#include "rclcpp/rclcpp.hpp"
#include "camera_stereo_calibration/camera_stereo_calibration_node.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraStereoCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}