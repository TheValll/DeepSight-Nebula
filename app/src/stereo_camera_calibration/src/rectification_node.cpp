#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
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
    show_debug_windows_ = declare_parameter<bool>("show_debug_windows", true);
    publish_depth_color_ = declare_parameter<bool>("publish_depth_color", true);
    publish_point_cloud_ = declare_parameter<bool>("publish_point_cloud", true);
    publish_metrics_ = declare_parameter<bool>("publish_metrics", false);
    num_disparities_ = declare_parameter<int>("num_disparities", 96);
    min_depth_meters_ = static_cast<float>(
      declare_parameter<double>("minimum_depth", 0.15));

    if (num_disparities_ <= 0 || num_disparities_ % 16 != 0) {
      throw std::runtime_error("num_disparities must be positive and divisible by 16");
    }

    left_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/stereo/left/image_rect", rclcpp::SensorDataQoS());
    depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/stereo/depth", rclcpp::SensorDataQoS());
    depth_color_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/stereo/depth/image_color", rclcpp::SensorDataQoS());
    depth_camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/stereo/depth/camera_info", rclcpp::SensorDataQoS());
    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/stereo/points", rclcpp::SensorDataQoS());
    metrics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pipeline/metrics", rclcpp::SensorDataQoS());

    load_calibration(calibration_file);
    run(camera_index);
  }

private:
  static constexpr int kStereoWidth = 2560;
  static constexpr int kStereoHeight = 720;
  static constexpr int kFramesPerSecond = 30;
  static constexpr double kDepthScale = 0.5;
  static constexpr int kMinDisparity = 0;
  static constexpr int kBlockSize = 5;
  static constexpr float kMaxDepthMeters = 2.0F;

  cv::Mat k1_;
  cv::Mat d1_;
  cv::Mat k2_;
  cv::Mat d2_;
  cv::Mat rotation_;
  cv::Mat translation_;
  bool show_debug_windows_ = true;
  bool publish_depth_color_ = true;
  bool publish_point_cloud_ = true;
  bool publish_metrics_ = false;
  int num_disparities_ = 96;
  float min_depth_meters_ = 0.15F;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr metrics_publisher_;

  static diagnostic_msgs::msg::KeyValue metric(
    const std::string & name, const double value)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = name;
    result.value = std::to_string(value);
    return result;
  }

  void publish_metrics(
    const std_msgs::msg::Header & header,
    const std::vector<diagnostic_msgs::msg::KeyValue> & values)
  {
    if (!publish_metrics_) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "stereo";
    status.message = "frame timings";
    status.hardware_id = "stereo_camera";
    status.values = values;
    message.status.push_back(status);
    metrics_publisher_->publish(message);
  }

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
    // V4L2 needs at least two buffers to capture at the requested frame rate.
    // A single buffer halves this camera from 30 FPS to about 15 FPS.
    camera.set(cv::CAP_PROP_BUFFERSIZE, 2);

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
      kMinDisparity, num_disparities_, kBlockSize,
      8 * kBlockSize * kBlockSize,
      32 * kBlockSize * kBlockSize,
      1, 63, 10, 100, 2, cv::StereoSGBM::MODE_SGBM_3WAY);

    if (show_debug_windows_) {
      cv::namedWindow("Stereo rectification", cv::WINDOW_NORMAL);
      cv::resizeWindow("Stereo rectification", 1600, 450);
      cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
      cv::resizeWindow("Disparity", 800, 450);
      cv::namedWindow("Depth (0.15 m - 2.0 m)", cv::WINDOW_NORMAL);
      cv::resizeWindow("Depth (0.15 m - 2.0 m)", 800, 450);
      RCLCPP_INFO(get_logger(), "Press 'q' or Escape to stop");
    }

    cv::Mat latest_frame;
    builtin_interfaces::msg::Time latest_stamp;
    double latest_camera_wait_ms = 0.0;
    double latest_frame_period_ms = -1.0;
    std::uint64_t latest_sequence = 0;
    std::mutex capture_mutex;
    std::condition_variable capture_ready;
    std::atomic_bool stop_capture{false};

    std::thread capture_thread(
      [&]() {
        auto previous_capture = std::chrono::steady_clock::time_point{};
        while (rclcpp::ok() && !stop_capture.load()) {
          const auto capture_start = std::chrono::steady_clock::now();
          cv::Mat captured_frame;
          if (!camera.read(captured_frame) || captured_frame.empty()) {
            RCLCPP_WARN(get_logger(), "Unable to capture a stereo frame");
            continue;
          }
          const auto capture_end = std::chrono::steady_clock::now();
          const double camera_wait_ms = std::chrono::duration<double, std::milli>(
            capture_end - capture_start).count();
          double frame_period_ms = -1.0;
          if (previous_capture != std::chrono::steady_clock::time_point{}) {
            frame_period_ms = std::chrono::duration<double, std::milli>(
              capture_end - previous_capture).count();
          }
          previous_capture = capture_end;

          {
            std::lock_guard<std::mutex> lock(capture_mutex);
            latest_frame = std::move(captured_frame);
            latest_stamp = now();
            latest_camera_wait_ms = camera_wait_ms;
            latest_frame_period_ms = frame_period_ms;
            ++latest_sequence;
          }
          capture_ready.notify_one();
        }
        capture_ready.notify_one();
      });

    cv::Mat stereo_frame;
    std::size_t frame_number = 0;
    std::uint64_t consumed_sequence = 0;
    while (rclcpp::ok()) {
      builtin_interfaces::msg::Time capture_stamp;
      double camera_wait_ms = 0.0;
      double frame_period_ms = -1.0;
      {
        std::unique_lock<std::mutex> lock(capture_mutex);
        capture_ready.wait(
          lock, [&]() {return !rclcpp::ok() || latest_sequence != consumed_sequence;});
        if (!rclcpp::ok()) {
          break;
        }
        stereo_frame = latest_frame;
        capture_stamp = latest_stamp;
        camera_wait_ms = latest_camera_wait_ms;
        frame_period_ms = latest_frame_period_ms;
        consumed_sequence = latest_sequence;
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
      const auto rectification_end = std::chrono::steady_clock::now();

      cv::Mat left_gray;
      cv::Mat right_gray;
      cv::cvtColor(left_depth_rectified, left_gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_depth_rectified, right_gray, cv::COLOR_BGR2GRAY);

      cv::Mat disparity_fixed;
      stereo_matcher->compute(left_gray, right_gray, disparity_fixed);

      cv::Mat disparity;
      disparity_fixed.convertTo(disparity, CV_32F, 1.0 / 16.0);
      const cv::Mat valid_disparity = disparity > kMinDisparity;

      cv::Mat disparity_color;
      if (show_debug_windows_) {
        cv::Mat disparity_gray;
        disparity.convertTo(disparity_gray, CV_8U, 255.0 / num_disparities_);
        disparity_gray.setTo(0, ~valid_disparity);
        cv::applyColorMap(disparity_gray, disparity_color, cv::COLORMAP_TURBO);
        disparity_color.setTo(cv::Scalar::all(0), ~valid_disparity);
      }

      cv::Mat points_3d;
      cv::reprojectImageTo3D(disparity, points_3d, depth_q, true);
      cv::Mat depth;
      cv::extractChannel(points_3d, depth, 2);
      const cv::Mat valid_depth =
        valid_disparity & (depth >= min_depth_meters_) & (depth <= kMaxDepthMeters);
      const auto depth_end = std::chrono::steady_clock::now();

      std_msgs::msg::Header image_header;
      image_header.stamp = capture_stamp;
      image_header.frame_id = "camera_left_optical_frame";
      left_image_publisher_->publish(
        *cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::BGR8, left_rectified).toImageMsg());
      depth_publisher_->publish(
        *cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::TYPE_32FC1, depth).toImageMsg());

      sensor_msgs::msg::CameraInfo depth_camera_info;
      depth_camera_info.header = image_header;
      depth_camera_info.width = depth_size.width;
      depth_camera_info.height = depth_size.height;
      depth_camera_info.distortion_model = "plumb_bob";
      depth_camera_info.d.assign(5, 0.0);
      depth_camera_info.k = {
        depth_p1.at<double>(0, 0), 0.0, depth_p1.at<double>(0, 2),
        0.0, depth_p1.at<double>(1, 1), depth_p1.at<double>(1, 2),
        0.0, 0.0, 1.0};
      depth_camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
          depth_camera_info.p[row * 4 + column] = depth_p1.at<double>(row, column);
        }
      }
      depth_camera_info_publisher_->publish(depth_camera_info);
      const auto image_publish_end = std::chrono::steady_clock::now();

      double point_cloud_ms = -1.0;
      if (
        publish_point_cloud_ && point_cloud_publisher_->get_subscription_count() > 0 &&
        frame_number % 3 == 0)
      {
        const auto point_cloud_start = std::chrono::steady_clock::now();
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = image_header;
        cloud.height = depth_size.height;
        cloud.width = depth_size.width;
        cloud.is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(static_cast<std::size_t>(depth_size.area()));

        sensor_msgs::PointCloud2Iterator<float> cloud_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> cloud_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> cloud_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> cloud_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> cloud_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> cloud_b(cloud, "b");
        const float invalid = std::numeric_limits<float>::quiet_NaN();

        for (int y = 0; y < depth_size.height; ++y) {
          for (int x = 0; x < depth_size.width; ++x) {
            const cv::Vec3f point = points_3d.at<cv::Vec3f>(y, x);
            const bool valid = valid_depth.at<uint8_t>(y, x) != 0;
            *cloud_x = valid ? point[0] : invalid;
            *cloud_y = valid ? point[1] : invalid;
            *cloud_z = valid ? point[2] : invalid;
            const cv::Vec3b color = left_depth_rectified.at<cv::Vec3b>(y, x);
            *cloud_r = color[2];
            *cloud_g = color[1];
            *cloud_b = color[0];
            ++cloud_x;
            ++cloud_y;
            ++cloud_z;
            ++cloud_r;
            ++cloud_g;
            ++cloud_b;
          }
        }
        point_cloud_publisher_->publish(cloud);
        point_cloud_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - point_cloud_start).count();
      }
      ++frame_number;

      cv::Mat depth_color;
      const bool needs_depth_color = show_debug_windows_ ||
        (publish_depth_color_ && depth_color_publisher_->get_subscription_count() > 0);
      if (needs_depth_color) {
        cv::Mat depth_clamped;
        cv::min(depth, kMaxDepthMeters, depth_clamped);
        cv::max(depth_clamped, min_depth_meters_, depth_clamped);
        cv::Mat depth_gray;
        depth_clamped.convertTo(
          depth_gray, CV_8U,
          -255.0 / (kMaxDepthMeters - min_depth_meters_),
          255.0 * kMaxDepthMeters / (kMaxDepthMeters - min_depth_meters_));
        depth_gray.setTo(0, ~valid_depth);
        cv::applyColorMap(depth_gray, depth_color, cv::COLORMAP_TURBO);
        depth_color.setTo(cv::Scalar::all(0), ~valid_depth);

        const cv::Point center(depth_size.width / 2, depth_size.height / 2);
        cv::drawMarker(
          depth_color, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 24, 2);
        const cv::Rect center_region(center.x - 10, center.y - 10, 21, 21);
        std::vector<float> center_depths;
        center_depths.reserve(center_region.area());
        for (int y = center_region.y; y < center_region.y + center_region.height; ++y) {
          for (int x = center_region.x; x < center_region.x + center_region.width; ++x) {
            const float value = depth.at<float>(y, x);
            if (value >= min_depth_meters_ && value <= kMaxDepthMeters) {
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

        if (publish_depth_color_ && depth_color_publisher_->get_subscription_count() > 0) {
          depth_color_publisher_->publish(
            *cv_bridge::CvImage(
              image_header, sensor_msgs::image_encodings::BGR8, depth_color).toImageMsg());
        }
      }

      cv::Mat display;
      if (show_debug_windows_) {
        cv::hconcat(left_rectified, right_rectified, display);
        for (int y = 40; y < display.rows; y += 40) {
          cv::line(
            display, cv::Point(0, y), cv::Point(display.cols - 1, y),
            cv::Scalar(0, 255, 0), 1);
        }
      }

      const auto processing_end = std::chrono::steady_clock::now();
      const double processing_ms = std::chrono::duration<double, std::milli>(
        processing_end - processing_start).count();
      if (show_debug_windows_) {
        char performance_label[64];
        std::snprintf(
          performance_label, sizeof(performance_label), "Processing: %.1f ms (%.1f FPS)",
          processing_ms, 1000.0 / processing_ms);
        cv::putText(
          display, performance_label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
          cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }

      std::vector<diagnostic_msgs::msg::KeyValue> metrics{
        metric("camera_wait_ms", camera_wait_ms),
        metric(
          "rectification_ms", std::chrono::duration<double, std::milli>(
            rectification_end - processing_start).count()),
        metric(
          "depth_ms", std::chrono::duration<double, std::milli>(
            depth_end - rectification_end).count()),
        metric(
          "image_publish_ms", std::chrono::duration<double, std::milli>(
            image_publish_end - depth_end).count()),
        metric(
          "visualization_ms", std::chrono::duration<double, std::milli>(
            processing_end - image_publish_end).count()),
        metric("total_processing_ms", processing_ms),
      };
      if (point_cloud_ms >= 0.0) {
        metrics.push_back(metric("point_cloud_ms", point_cloud_ms));
      }
      if (frame_period_ms >= 0.0) {
        metrics.push_back(metric("frame_period_ms", frame_period_ms));
      }
      publish_metrics(image_header, metrics);

      if (show_debug_windows_) {
        cv::imshow("Stereo rectification", display);
        cv::imshow("Disparity", disparity_color);
        cv::imshow("Depth (0.15 m - 2.0 m)", depth_color);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
          break;
        }
      }
    }

    stop_capture.store(true);
    capture_ready.notify_one();
    if (capture_thread.joinable()) {
      capture_thread.join();
    }
    camera.release();
    if (show_debug_windows_) {
      cv::destroyAllWindows();
    }
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
