#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class RectificationNode : public rclcpp::Node
{
public:
  RectificationNode()
  : Node("stereo_rectification_node")
  {
    const auto calibration_file =
      declare_parameter<std::string>("calibration_file", "calibration/stereo_calib.xml");
    const auto camera_index = declare_parameter<int>("camera_index", 0);

    left_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/stereo/left/image_rect", rclcpp::SensorDataQoS());
    depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/stereo/depth", rclcpp::SensorDataQoS());

    load_calibration(calibration_file);
    run(camera_index);
  }

private:
  static constexpr int kStereoWidth = 2560;
  static constexpr int kStereoHeight = 720;
  static constexpr int kFramesPerSecond = 30;
  static constexpr double kDepthScale = 0.5;
  static constexpr int kMinDisparity = 0;
  static constexpr int kNumDisparities = 96;
  static constexpr int kBlockSize = 5;
  static constexpr float kMinDepthMeters = 0.15F;
  static constexpr float kMaxDepthMeters = 2.0F;

  cv::Mat k1_;
  cv::Mat d1_;
  cv::Mat k2_;
  cv::Mat d2_;
  cv::Mat rotation_;
  cv::Mat translation_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;

  void load_calibration(const std::string & path)
  {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
      throw std::runtime_error("Unable to open calibration file: " + path);
    }

    storage["K1"] >> k1_;
    storage["D1"] >> d1_;
    storage["K2"] >> k2_;
    storage["D2"] >> d2_;
    storage["R"] >> rotation_;
    storage["T"] >> translation_;

    if (
      k1_.empty() || d1_.empty() || k2_.empty() || d2_.empty() ||
      rotation_.empty() || translation_.empty())
    {
      throw std::runtime_error("Calibration file is incomplete: " + path);
    }

    RCLCPP_INFO(get_logger(), "Loaded stereo calibration from %s", path.c_str());
  }

  void run(int camera_index)
  {
    cv::VideoCapture camera(camera_index, cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, kStereoWidth);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, kStereoHeight);
    camera.set(cv::CAP_PROP_FPS, kFramesPerSecond);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!camera.isOpened()) {
      throw std::runtime_error("Unable to open stereo camera");
    }

    const int actual_width = static_cast<int>(std::lround(camera.get(cv::CAP_PROP_FRAME_WIDTH)));
    const int actual_height = static_cast<int>(std::lround(camera.get(cv::CAP_PROP_FRAME_HEIGHT)));
    const double actual_fps = camera.get(cv::CAP_PROP_FPS);

    if (actual_width != kStereoWidth || actual_height != kStereoHeight) {
      throw std::runtime_error(
              "Unexpected camera resolution: " + std::to_string(actual_width) + "x" +
              std::to_string(actual_height));
    }

    RCLCPP_INFO(
      get_logger(), "Camera stream: %dx%d at %.1f FPS", actual_width, actual_height,
      actual_fps);

    const cv::Size eye_size(kStereoWidth / 2, kStereoHeight);
    cv::Mat r1;
    cv::Mat r2;
    cv::Mat p1;
    cv::Mat p2;
    cv::Mat q;
    cv::stereoRectify(
      k1_, d1_, k2_, d2_, eye_size, rotation_, translation_, r1, r2, p1, p2, q,
      cv::CALIB_ZERO_DISPARITY, 0.0, eye_size);

    cv::Mat left_map_x;
    cv::Mat left_map_y;
    cv::Mat right_map_x;
    cv::Mat right_map_y;
    cv::initUndistortRectifyMap(
      k1_, d1_, r1, p1, eye_size, CV_32FC1, left_map_x, left_map_y);
    cv::initUndistortRectifyMap(
      k2_, d2_, r2, p2, eye_size, CV_32FC1, right_map_x, right_map_y);

    const cv::Size depth_size(
      static_cast<int>(eye_size.width * kDepthScale),
      static_cast<int>(eye_size.height * kDepthScale));
    cv::Mat depth_k1 = k1_.clone();
    cv::Mat depth_k2 = k2_.clone();
    for (cv::Mat * camera_matrix : {&depth_k1, &depth_k2}) {
      camera_matrix->at<double>(0, 0) *= kDepthScale;
      camera_matrix->at<double>(1, 1) *= kDepthScale;
      camera_matrix->at<double>(0, 2) *= kDepthScale;
      camera_matrix->at<double>(1, 2) *= kDepthScale;
    }

    cv::Mat depth_r1;
    cv::Mat depth_r2;
    cv::Mat depth_p1;
    cv::Mat depth_p2;
    cv::Mat depth_q;
    cv::stereoRectify(
      depth_k1, d1_, depth_k2, d2_, depth_size, rotation_, translation_, depth_r1,
      depth_r2, depth_p1, depth_p2, depth_q, cv::CALIB_ZERO_DISPARITY, 0.0, depth_size);

    cv::Mat depth_left_map_x;
    cv::Mat depth_left_map_y;
    cv::Mat depth_right_map_x;
    cv::Mat depth_right_map_y;
    cv::initUndistortRectifyMap(
      k1_, d1_, depth_r1, depth_p1, depth_size, CV_32FC1, depth_left_map_x,
      depth_left_map_y);
    cv::initUndistortRectifyMap(
      k2_, d2_, depth_r2, depth_p2, depth_size, CV_32FC1, depth_right_map_x,
      depth_right_map_y);

    auto stereo_matcher = cv::StereoSGBM::create(
      kMinDisparity, kNumDisparities, kBlockSize,
      8 * kBlockSize * kBlockSize,
      32 * kBlockSize * kBlockSize,
      1, 63, 10, 100, 2, cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::namedWindow("Stereo rectification", cv::WINDOW_NORMAL);
    cv::resizeWindow("Stereo rectification", 1600, 450);
    cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
    cv::resizeWindow("Disparity", 800, 450);
    cv::namedWindow("Depth (0.15 m - 2.0 m)", cv::WINDOW_NORMAL);
    cv::resizeWindow("Depth (0.15 m - 2.0 m)", 800, 450);
    RCLCPP_INFO(get_logger(), "Press 'q' or Escape to stop");

    cv::Mat stereo_frame;
    while (rclcpp::ok()) {
      if (!camera.read(stereo_frame) || stereo_frame.empty()) {
        RCLCPP_WARN(get_logger(), "Unable to capture a stereo frame");
        continue;
      }

      if (stereo_frame.size() != cv::Size(kStereoWidth, kStereoHeight)) {
        RCLCPP_ERROR(
          get_logger(), "Received an unexpected frame size: %dx%d", stereo_frame.cols,
          stereo_frame.rows);
        break;
      }

      const auto processing_start = std::chrono::steady_clock::now();

      const cv::Mat left = stereo_frame(cv::Rect(0, 0, eye_size.width, eye_size.height));
      const cv::Mat right =
        stereo_frame(cv::Rect(eye_size.width, 0, eye_size.width, eye_size.height));

      cv::Mat left_rectified;
      cv::Mat right_rectified;
      cv::remap(left, left_rectified, left_map_x, left_map_y, cv::INTER_LINEAR);
      cv::remap(right, right_rectified, right_map_x, right_map_y, cv::INTER_LINEAR);

      cv::Mat left_depth_rectified;
      cv::Mat right_depth_rectified;
      cv::remap(
        left, left_depth_rectified, depth_left_map_x, depth_left_map_y, cv::INTER_LINEAR);
      cv::remap(
        right, right_depth_rectified, depth_right_map_x, depth_right_map_y,
        cv::INTER_LINEAR);

      cv::Mat left_gray;
      cv::Mat right_gray;
      cv::cvtColor(left_depth_rectified, left_gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_depth_rectified, right_gray, cv::COLOR_BGR2GRAY);

      cv::Mat disparity_fixed;
      stereo_matcher->compute(left_gray, right_gray, disparity_fixed);

      cv::Mat disparity;
      disparity_fixed.convertTo(disparity, CV_32F, 1.0 / 16.0);
      const cv::Mat valid_disparity = disparity > kMinDisparity;

      cv::Mat disparity_gray;
      disparity.convertTo(disparity_gray, CV_8U, 255.0 / kNumDisparities);
      disparity_gray.setTo(0, ~valid_disparity);
      cv::Mat disparity_color;
      cv::applyColorMap(disparity_gray, disparity_color, cv::COLORMAP_TURBO);
      disparity_color.setTo(cv::Scalar::all(0), ~valid_disparity);

      cv::Mat points_3d;
      cv::reprojectImageTo3D(disparity, points_3d, depth_q, true);
      cv::Mat depth;
      cv::extractChannel(points_3d, depth, 2);
      const cv::Mat valid_depth =
        valid_disparity & (depth >= kMinDepthMeters) & (depth <= kMaxDepthMeters);

      std_msgs::msg::Header image_header;
      image_header.stamp = now();
      image_header.frame_id = "camera_left_lens_link";
      left_image_publisher_->publish(
        *cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::BGR8, left_rectified).toImageMsg());
      depth_publisher_->publish(
        *cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::TYPE_32FC1, depth).toImageMsg());

      cv::Mat depth_clamped;
      cv::min(depth, kMaxDepthMeters, depth_clamped);
      cv::max(depth_clamped, kMinDepthMeters, depth_clamped);
      cv::Mat depth_gray;
      depth_clamped.convertTo(
        depth_gray, CV_8U,
        -255.0 / (kMaxDepthMeters - kMinDepthMeters),
        255.0 * kMaxDepthMeters / (kMaxDepthMeters - kMinDepthMeters));
      depth_gray.setTo(0, ~valid_depth);
      cv::Mat depth_color;
      cv::applyColorMap(depth_gray, depth_color, cv::COLORMAP_TURBO);
      depth_color.setTo(cv::Scalar::all(0), ~valid_depth);

      const cv::Point center(depth_size.width / 2, depth_size.height / 2);
      cv::drawMarker(depth_color, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 24, 2);
      const cv::Rect center_region(center.x - 10, center.y - 10, 21, 21);
      std::vector<float> center_depths;
      center_depths.reserve(center_region.area());
      for (int y = center_region.y; y < center_region.y + center_region.height; ++y) {
        for (int x = center_region.x; x < center_region.x + center_region.width; ++x) {
          const float value = depth.at<float>(y, x);
          if (value >= kMinDepthMeters && value <= kMaxDepthMeters) {
            center_depths.push_back(value);
          }
        }
      }
      if (!center_depths.empty()) {
        const auto middle = center_depths.begin() + center_depths.size() / 2;
        std::nth_element(center_depths.begin(), middle, center_depths.end());
        char label[32];
        std::snprintf(label, sizeof(label), "Center: %.3f m", *middle);
        cv::putText(
          depth_color, label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
          cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      }

      cv::Mat display;
      cv::hconcat(left_rectified, right_rectified, display);
      for (int y = 40; y < display.rows; y += 40) {
        cv::line(display, cv::Point(0, y), cv::Point(display.cols - 1, y), cv::Scalar(0, 255, 0), 1);
      }

      const auto processing_end = std::chrono::steady_clock::now();
      const double processing_ms = std::chrono::duration<double, std::milli>(
        processing_end - processing_start).count();
      char performance_label[64];
      std::snprintf(
        performance_label, sizeof(performance_label), "Processing: %.1f ms (%.1f FPS)",
        processing_ms, 1000.0 / processing_ms);
      cv::putText(
        display, performance_label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

      cv::imshow("Stereo rectification", display);
      cv::imshow("Disparity", disparity_color);
      cv::imshow("Depth (0.15 m - 2.0 m)", depth_color);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 27) {
        break;
      }
    }

    camera.release();
    cv::destroyAllWindows();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RectificationNode>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_rectification_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
