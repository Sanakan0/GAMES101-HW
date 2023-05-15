cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR "C:/Users/SA-NA-KAN/Downloads/opencv/build")

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(../../3rdparty)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
