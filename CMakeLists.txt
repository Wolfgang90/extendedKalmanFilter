cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(sources
    src/main.cpp
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/tools.cpp)

project(ExtendedKF)

add_executable(ExtendedKF ${sources})
