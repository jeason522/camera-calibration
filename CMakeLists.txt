cmake_minimum_required(VERSION 3.10)
project(camera_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})
target_compile_options(calibrate PRIVATE -Wall -Wextra)
