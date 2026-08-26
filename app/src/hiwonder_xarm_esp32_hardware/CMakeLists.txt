cmake_minimum_required(VERSION 3.16)
project(hiwonder_xarm_esp32_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../utils/rust_firmware_driver")

add_library(${PROJECT_NAME} SHARED
  src/hiwonder_system.cpp
  "${DRIVER_ROOT}/src/hiwonder_xarm_rust_driver.cpp"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${DRIVER_ROOT}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} Boost::system)

add_executable(servo_calibration_cli
  src/servo_calibration_cli.cpp
  "${DRIVER_ROOT}/src/hiwonder_xarm_rust_driver.cpp"
)
target_compile_features(servo_calibration_cli PUBLIC cxx_std_17)
target_include_directories(servo_calibration_cli PRIVATE
  "${DRIVER_ROOT}/include"
)
target_link_libraries(servo_calibration_cli Boost::system)

pluginlib_export_plugin_description_file(
  hardware_interface hiwonder_xarm_esp32_hardware.xml)

install(TARGETS ${PROJECT_NAME} servo_calibration_cli
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES
  "${DRIVER_ROOT}/include/hiwonder_xarm_rust_driver.hpp"
  DESTINATION include
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_calibration test/test_joint_calibration.cpp)
  target_link_libraries(test_joint_calibration ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
