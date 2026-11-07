cmake_minimum_required(VERSION 3.22)
project(hiwonder_xarm_esp32_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# Preserve the ROS dylib search path under macOS SIP.
find_package(Python3 3.12 EXACT REQUIRED COMPONENTS Interpreter)
set(ground_source "${CMAKE_CURRENT_SOURCE_DIR}/scripts/add_ground.py")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ground_source}")
file(READ "${ground_source}" ground_script)
string(REGEX REPLACE "^#![^\n]*" "#!${Python3_EXECUTABLE}" ground_script "${ground_script}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/add_ground" "${ground_script}")
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/add_ground" PERMISSIONS
  OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/add_ground"
  DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
