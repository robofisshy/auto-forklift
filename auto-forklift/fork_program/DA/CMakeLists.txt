cmake_minimum_required(VERSION 3.5)
project(DA_Init)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

set(SOURCE_FILES main.cpp jetsonGPIO.cpp motorControl.cpp)

add_executable(DA_Init ${SOURCE_FILES})
