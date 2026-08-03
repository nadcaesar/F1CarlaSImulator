cmake_minimum_required(VERSION 3.10)
project (F1Simulator CXX)
set (CMAKE_CXX_STANDARD 17)
add_executable(f1sim src/main.cpp src/vehicle/vehicle.cpp)