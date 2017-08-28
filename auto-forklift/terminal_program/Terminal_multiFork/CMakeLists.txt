cmake_minimum_required(VERSION 3.6)
project(Terminal_multiFork)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(SOURCE_FILES main.cpp src/Map/map.cpp src/Listener/Listener.cpp src/Terminal/Terminal.cpp src/Socket/Socket.cpp)
add_executable(Terminal_multiFork ${SOURCE_FILES})
target_link_libraries(Terminal_multiFork libpthread.so ${OpenCV_LIBS} ${Boost_LIBRARIES})