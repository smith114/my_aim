cmake_minimum_required(VERSION 3.16)
project(my_aim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED)

include_directories(include)

add_executable(my_aim src/main.cpp)
target_link_libraries(my_aim PRIVATE ${OpenCV_LIBS} openvino::runtime)

add_executable(yolo_test src/yolo_test.cpp)
target_link_libraries(yolo_test PRIVATE ${OpenCV_LIBS} openvino::runtime)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS})