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
# 重要发现(2026-10-16 在 NUC 上实测):
#   项目自带的 thirdparty/hikrobot/lib/libMvCameraControl.so 只有 6.9 MB,
#   它**只是 API 外壳**, 不带 USB3 Vision 传输层, MV_CC_EnumDevices 返回
#   ret=0x80000006 且 nDeviceNum=0 —— 明明 lsusb 能看到相机却枚举不到。
#   系统安装的 /opt/MVS/lib/64 是完整包(带 libMvUsb3vTL.so 和 MvProducerU3V.cti),
#   同一个程序链上它 ret=0 nDeviceNum=1, 正常。
# 所以优先用系统 MVS 的动态库; 头文件仍可用项目自带的(SDK 版本同是 4.7.0.3)。
set(HIK_SYS_LIB_DIR "/opt/MVS/lib/64" CACHE PATH "系统 MVS 动态库目录")
set(HIK_SYS_LIB     "${HIK_SYS_LIB_DIR}/libMvCameraControl.so")
set(HIK_BUNDLED_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hikrobot/lib/libMvCameraControl.so")
set(HIK_INCLUDE     "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hikrobot/include")

if(EXISTS ${HIK_SYS_LIB})
  set(HIK_LINK_LIB ${HIK_SYS_LIB})
  set(HIK_RPATH     ${HIK_SYS_LIB_DIR})
  message(STATUS "海康 SDK: 使用系统 MVS (${HIK_SYS_LIB}) —— 带 USB3 传输层")
elseif(EXISTS ${HIK_BUNDLED_LIB})
  set(HIK_LINK_LIB ${HIK_BUNDLED_LIB})
  set(HIK_RPATH     "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hikrobot/lib")
  message(WARNING
    "海康 SDK: 未找到系统 MVS (${HIK_SYS_LIB}), 回退到项目自带版本。\n"
    "  项目自带版本缺少 USB3 Vision 传输层, 会枚举不到相机(返回 nDeviceNum=0)。\n"
    "  请安装完整 MVS SDK (默认装到 /opt/MVS), 或用 -DHIK_SYS_LIB_DIR=... 指定路径。")
else()
  message(FATAL_ERROR "找不到 libMvCameraControl.so, 请安装海康 MVS SDK")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${HIK_INCLUDE})

# 所有需要海康相机 + 串口的可执行文件共用的链接与 rpath 设置
function(aim_link target)
  target_link_libraries(${target} PRIVATE
    ${OpenCV_LIBS}
    ${HIK_LINK_LIB}
    Threads::Threads)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH   "${HIK_RPATH}"
    INSTALL_RPATH "${HIK_RPATH}")
endfunction()

# ---------------------------------------------------------------- 主程序
add_executable(my_aim src/main.cpp)
aim_link(my_aim)

# ---------------------------------------------------------------- 标定工具
add_executable(calibrate src/calibrate.cpp)
aim_link(calibrate)

# ---------------------------------------------------------------- 云台诊断
# 不用手推云台就能读出当前实测姿态、单步走、或软件扫描找限位
add_executable(gimbal_test src/gimbal_test.cpp)
aim_link(gimbal_test)

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
