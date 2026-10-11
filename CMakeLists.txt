cmake_minimum_required(VERSION 3.16)
project(my_aim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(my_aim src/main.cpp)
target_link_libraries(my_aim PRIVATE ${OpenCV_LIBS})