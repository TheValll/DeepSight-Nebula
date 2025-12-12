#include "rclcpp/rclcpp.hpp"

class CameraStereoCalibrationNode : public rclcpp::Node{
    public:
        CameraStereoCalibrationNode(): Node("camera_stereo_calibration_node"){
            RCLCPP_INFO(this->get_logger(), "Test");
        }
    private :
}