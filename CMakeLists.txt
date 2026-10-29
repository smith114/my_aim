cmake_minimum_required(VERSION 3.16)
project(my_aim CXX)

# 导出 compile_commands.json: 让 VSCode 的 IntelliSense 用**真实编译参数**,
# 而不是靠猜 include 路径。学习代码时跳转/补全才准。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------- 靶标预设
# 说明: armor.hpp 里的"宽高比"是 **灯条间距 / 灯条长度** 的像素比值。
# 它只在"灯条接近满板高"时才等于板子的真实宽高比。实机靶标的灯条比板高短,
# 所以实测值偏大 —— 这也是为什么不能直接套 140/125=1.12。
#
# 实机标定(10-16, 正视):
#   正确配对(夹着白色数字"2"的那两根灯条): aspect ≈ 2.84
#   跨板误配(把隔壁板的灯条配进来):       aspect >= 4
#   -> 上限取 3.5 收得下真板、拦得住跨板。
#
# 若以后确认靶标灯条是接近满板高的(比如标准 RM 装甲板), 开下面这个开关,
# 上限收紧到 1.6, 约束会强得多。
#
#   cmake -S . -B build -DUSE_STANDARD_TARGET=ON
option(USE_STANDARD_TARGET "靶标灯条接近满板高时收紧宽高比约束" OFF)
if(USE_STANDARD_TARGET)
  add_compile_definitions(PAIR_ASPECT_MAX=1.6f)
  message(STATUS "靶标预设: 灯条接近满板高 (aspect 上限 1.6, 强约束)")
else()
  add_compile_definitions(PAIR_ASPECT_MAX=3.5f)
  message(STATUS "靶标预设: 通用 (aspect 上限 3.5, 按实机标定)")
endif()

# 装甲板宽高比下限。同济 min_armor_ratio=1, 我们原来 0.25 太松(会放进竖长条误配)。
# 留一点余量取 0.6: 允许斜视时板变窄, 但不允许"高比宽大 4 倍"的怪形状。
add_compile_definitions(PAIR_ASPECT_MIN=0.6f)

# (Eigen 已移除: 同济 EKF 子系统整体删除, 不再需要 Eigen)
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
# OpenVINO 必须在 add_executable(my_aim) **之前** 探测, 因为主程序要按它决定
# 是否编入 YOLO 检测分支 (HAVE_OPENVINO 宏), 以及是否链接 openvino::runtime。
find_package(OpenVINO QUIET)

# 系统安装包 (官方 tarball 装到 /opt/intel/openvino_*) 需要先 source setupvars.sh
# 才会设 OpenVINO_DIR。但要求所有人"先在每个新 shell 里 source 再 cmake"太容易踩,
# 所以这里主动去标准安装位置找一遍。实测 NUC 上装了 2024.6.0 却因没 source 而
# 报告"没有 OpenVINO", 导致 yolo_test 没编、主程序也没有 --yolo。
if(NOT OpenVINO_FOUND)
  file(GLOB _ov_sys_dirs "/opt/intel/openvino_*/runtime/cmake")
  list(SORT _ov_sys_dirs ORDER DESCENDING)     # 多版本时优先新的
  foreach(_d ${_ov_sys_dirs})
    if(EXISTS "${_d}/OpenVINOConfig.cmake")
      list(APPEND CMAKE_PREFIX_PATH "${_d}")
      find_package(OpenVINO QUIET)
      if(OpenVINO_FOUND)
        message(STATUS "OpenVINO: 从系统安装位置找到 (${_d})")
        break()
      endif()
    endif()
  endforeach()
endif()

# pip 安装的 openvino 不会进系统 CMAKE_PREFIX_PATH, 但包内自带 cmake 配置。
# 这里主动问一下 python 它的位置, 让 `pip install openvino` 也能被找到 ——
# 否则同一个包 python 能用、CMake 说没有, 很难理解。
if(NOT OpenVINO_FOUND)
  execute_process(
    COMMAND python3 -c "import openvino,os;print(os.path.join(os.path.dirname(openvino.__file__),'cmake'))"
    OUTPUT_VARIABLE _ov_pip_cmake OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(_ov_pip_cmake AND EXISTS "${_ov_pip_cmake}")
    list(APPEND CMAKE_PREFIX_PATH "${_ov_pip_cmake}")
    find_package(OpenVINO QUIET)
    if(OpenVINO_FOUND)
      message(STATUS "OpenVINO: 从 pip 安装位置找到 (${_ov_pip_cmake})")
    endif()
  endif()
endif()

add_executable(my_aim src/main.cpp)
aim_link(my_aim)
if(OpenVINO_FOUND)
  target_compile_definitions(my_aim PRIVATE HAVE_OPENVINO=1)
  target_link_libraries(my_aim PRIVATE openvino::runtime)
  message(STATUS "YOLO 检测: 已编入 my_aim (OpenVINO 可用)")
else()
  message(STATUS "YOLO 检测: my_aim 未编入 (没找到 OpenVINO)")
endif()

# ---------------------------------------------------------------- 伺服方向自动标定
# 实机上自动测出 SERVO_DIR_YAW / SERVO_DIR_PITCH, 不靠猜。
# 判据: 锁住目标后把命令角挪一小步, 看 |theta| 变小还是变大。
add_executable(servo_calib src/servo_calib.cpp)
aim_link(servo_calib)
if(OpenVINO_FOUND)
  target_compile_definitions(servo_calib PRIVATE HAVE_OPENVINO=1)
  target_link_libraries(servo_calib PRIVATE openvino::runtime)
else()
  message(WARNING "servo_calib 需要 OpenVINO, 未编入")
endif()

# ---------------------------------------------------------------- 可视化调试界面
add_executable(aim_gui src/aim_gui.cpp)
aim_link(aim_gui)
if(OpenVINO_FOUND)
  target_compile_definitions(aim_gui PRIVATE HAVE_OPENVINO=1)
  target_link_libraries(aim_gui PRIVATE openvino::runtime)
endif()

# (同宗 EKF 子系统已移除 —— 未接入主流程, 见标签 before-cleanup-20261029)

# ---------------------------------------------------------------- 标定工具
add_executable(calibrate src/calibrate.cpp)
aim_link(calibrate)

# ---------------------------------------------------------------- 云台诊断
# 不用手推云台就能读出当前实测姿态、单步走、或软件扫描找限位
add_executable(gimbal_test src/gimbal_test.cpp)
aim_link(gimbal_test)

# ---------------------------------------------------------------- 实时成像查看
# 窗口实时显示相机画面 + YOLO 检测标注; --headless 时定期存图
#
# **必须和 my_aim 一样编入 YOLO**: 否则窗口里显示的是另一套检测器的结果,
# "窗口能看到但主程序没反应"这种不一致最难排查。之前漏了这一步, 现场就撞上了。
add_executable(cam_view src/cam_view.cpp)
aim_link(cam_view)
if(OpenVINO_FOUND)
  target_compile_definitions(cam_view PRIVATE HAVE_OPENVINO=1)
  target_link_libraries(cam_view PRIVATE openvino::runtime)
  message(STATUS "YOLO 检测: 已编入 cam_view (OpenVINO 可用)")
else()
  message(STATUS "YOLO 检测: cam_view 未编入 (没找到 OpenVINO)")
endif()

# ---------------------------------------------------------------- 离线推理自检
# 用图片验证 YOLO/OpenVINO 环境是否可用, 不依赖相机
if(OpenVINO_FOUND)
  add_executable(yolo_test src/yolo_test.cpp)
  target_link_libraries(yolo_test PRIVATE ${OpenCV_LIBS} openvino::runtime Threads::Threads)
else()
  message(STATUS "未找到 OpenVINO, 跳过 yolo_test")
endif()

# ---------------------------------------------------------------- 测试
enable_testing()
add_subdirectory(tests)
