cmake_minimum_required (VERSION 2.8.12)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_executable (Assignment0 assignment0.cpp)
add_executable (Main main.cpp)