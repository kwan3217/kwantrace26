cmake_minimum_required(VERSION 4.1)
project(kwantrace26)

set(CMAKE_CXX_STANDARD 26)

add_executable(kwantrace26 main.cpp
        include/kwantrace/kwantrace.hpp
        include/kwantrace/math.hpp
        include/eigen/Eigen/Dense
        include/kwantrace/pdvector.hpp
)

target_include_directories(kwantrace26 PRIVATE include)

