cmake_minimum_required(VERSION 3.22)
project(sample4)

set(CMAKE_CXX_STANDARD 14)
include_directories("C:/Users/wd230/CGProgramming/eigen-3.4.0")

add_executable(sample4 main.cpp Camera.cpp Camera.h)
target_link_libraries(sample4 libopengl32.a libglu32.a libfreeglut.a)
