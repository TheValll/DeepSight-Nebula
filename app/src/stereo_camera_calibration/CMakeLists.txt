cmake_minimum_required(VERSION 3.22)
project(stereo_camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# ROS Jazzy/cv_bridge use OpenCV 4; Homebrew also provides OpenCV 5.
if(APPLE AND NOT OpenCV_DIR)
  execute_process(COMMAND brew --prefix opencv@4
    OUTPUT_VARIABLE OPENCV_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE OPENCV_BREW_RESULT)
  if(OPENCV_BREW_RESULT EQUAL 0)
    set(OpenCV_DIR "${OPENCV_PREFIX}/lib/cmake/opencv4")
  endif()
endif()
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d highgui videoio)
find_package(Threads REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibration_exe src/calibration_node.cpp)
add_executable(rectification_exe src/rectification_node.cpp)
target_compile_features(calibration_exe PRIVATE cxx_std_17)
target_compile_features(rectification_exe PRIVATE cxx_std_17)

ament_target_dependencies(calibration_exe rclcpp)
ament_target_dependencies(rectification_exe cv_bridge diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS
  calibration_exe
  rectification_exe
  DESTINATION lib/${PROJECT_NAME}
)

# A direct interpreter path preserves DYLD_LIBRARY_PATH under macOS SIP.
find_package(Python3 3.12 EXACT REQUIRED COMPONENTS Interpreter)
function(install_python_node source executable)
  set(source_path "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source_path}")
  file(READ "${source_path}" script)
  string(REGEX REPLACE "^#![^\n]*" "#!${Python3_EXECUTABLE}" script "${script}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${executable}" "${script}")
  file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/${executable}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${executable}"
    DESTINATION lib/${PROJECT_NAME})
endfunction()

install_python_node(scripts/yolo_detection_node.py yolo_detection_node)
install_python_node(scripts/camera_desk_calibration_node.py camera_desk_calibration)
install_python_node(scripts/ball_goal_preview_node.py ball_goal_preview_node)
install_python_node(scripts/ball_pick_node.py ball_pick_node)
install_python_node(scripts/pick_client.py pick)
install_python_node(scripts/pipeline_benchmark_node.py pipeline_benchmark_node)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(calibration_exe
  ${OpenCV_LIBS}
)

target_link_libraries(rectification_exe
  ${OpenCV_LIBS}
  Threads::Threads
)

ament_package()
