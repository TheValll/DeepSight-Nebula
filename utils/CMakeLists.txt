cmake_minimum_required(VERSION 3.10)

project(StereoCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
add_executable(calibration stereo_calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})