cmake_minimum_required(VERSION 3.5)
project(stardraw_forklift)

set(CMAKE_CXX_FLAGS "-g")
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

file(GLOB location_src src/location/*.cpp)
file(GLOB thread_src src/multi_thread/*.cpp)
file(GLOB utils_src src/utils/*.cpp)
file(GLOB motion_src src/motor_control/*.cpp)
file(GLOB detect_src src/detect/*.cpp)
file(GLOB fork_src src/forklift/*.cpp)

set(SOURCE_FILES src/main.cpp ${location_src} ${motor_src} ${thread_src} ${utils_src} ${motion_src} ${detect_src} ${fork_src})
add_executable(stardraw_forklift ${SOURCE_FILES})
target_link_libraries(stardraw_forklift ${OpenCV_LIBS} libpthread.so ${Boost_LIBRARIES} libcontrolcan.so)
