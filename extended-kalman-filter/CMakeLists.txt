cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ekf_demo
    src/main.cpp
    src/ekf.cpp
    src/tools.cpp
)

target_include_directories(ekf_demo PRIVATE
    "C:/eigen-5.0.0"
)