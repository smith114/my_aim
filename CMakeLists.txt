cmake_minimum_required(VERSION 3.16)
project(my_aim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------- 海康 MVS SDK
# 项目自带 SDK 头文件与 .so, 用 RPATH 定位, 不需要装到系统里
set(HIK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hikrobot)
set(HIK_INCLUDE ${HIK_DIR}/include)
set(HIK_LIB     ${HIK_DIR}/lib)
if(NOT EXISTS ${HIK_LIB}/libMvCameraControl.so)
  message(FATAL_ERROR "缺少 ${HIK_LIB}/libMvCameraControl.so")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${HIK_INCLUDE})

# 所有需要海康相机 + 串口的可执行文件共用的链接与 rpath 设置
function(aim_link target)
  target_link_libraries(${target} PRIVATE
    ${OpenCV_LIBS}
    ${HIK_LIB}/libMvCameraControl.so
    Threads::Threads)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH   "${HIK_LIB}"
    INSTALL_RPATH "${HIK_LIB}")
endfunction()

# ---------------------------------------------------------------- 主程序
add_executable(my_aim src/main.cpp)
aim_link(my_aim)

# ---------------------------------------------------------------- 标定工具
add_executable(calibrate src/calibrate.cpp)
aim_link(calibrate)

# ---------------------------------------------------------------- 离线推理自检
# 用图片验证 YOLO/OpenVINO 环境是否可用, 不依赖相机
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
  add_executable(yolo_test src/yolo_test.cpp)
  target_link_libraries(yolo_test PRIVATE ${OpenCV_LIBS} openvino::runtime Threads::Threads)
else()
  message(STATUS "未找到 OpenVINO, 跳过 yolo_test (主程序不依赖它)")
endif()

# ---------------------------------------------------------------- 测试
enable_testing()
add_subdirectory(tests)
