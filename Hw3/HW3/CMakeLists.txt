cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR "D:/games101/opencv/build")
#set(OpenCV_DIR "C:\\Users\\SA-NA-KAN\\Downloads\\opencv\\build")

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)


include_directories(../../3rdparty)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h Profiler.hpp Profiler.cpp Postprocess.hpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
target_compile_options(Rasterizer PUBLIC)
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)