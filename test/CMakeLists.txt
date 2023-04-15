cmake_minimum_required(VERSION 3.10)
project(Test)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(Test main.cpp )
target_link_libraries(Test ${OpenCV_LIBRARIES})
