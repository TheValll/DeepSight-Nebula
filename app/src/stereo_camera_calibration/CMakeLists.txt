cmake_minimum_required(VERSION 3.8)
project(stereo_camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibration_exe src/calibration_node.cpp)
add_executable(rectification_exe src/rectification_node.cpp)

ament_target_dependencies(calibration_exe rclcpp)
ament_target_dependencies(rectification_exe cv_bridge rclcpp sensor_msgs)

install(TARGETS
  calibration_exe
  rectification_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/yolo_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detection_node
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(calibration_exe
  ${OpenCV_LIBS}
)

target_link_libraries(rectification_exe
  ${OpenCV_LIBS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
