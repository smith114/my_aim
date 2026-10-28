# 测试: 全部离线可跑, 不需要相机/下位机/实车
#   cmake --build build && ctest --test-dir build --output-on-failure

set(TEST_TARGETS
  test_protocol
  test_solver
  test_armor
  test_tracker
  test_closed_loop
  gimbal_edge
  test_kf
  test_kf3d
  test_spin_ekf
  test_frame
)

foreach(t ${TEST_TARGETS})
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE ${OpenCV_LIBS} Threads::Threads)
  target_include_directories(${t} PRIVATE ${CMAKE_SOURCE_DIR}/include)
  # 测试用正向图: 合成靶标是正的, 实车的 180 度旋转会让坐标镜像
  target_compile_definitions(${t} PRIVATE IMG_DIR_OVERRIDE=0)
  add_test(NAME ${t} COMMAND ${t})
  set_tests_properties(${t} PROPERTIES TIMEOUT 300)
endforeach()

# 诊断工具(不是测试, 不注册给 ctest): 生成合成图 / 打印检测中间结果
add_executable(synth_dump synth_dump.cpp)
target_link_libraries(synth_dump PRIVATE ${OpenCV_LIBS})
target_include_directories(synth_dump PRIVATE ${CMAKE_SOURCE_DIR}/include)

# detect_probe 支持 --cam / --hold, 需要相机 SDK 与串口
set(HIK_INCLUDE ${CMAKE_SOURCE_DIR}/thirdparty/hikrobot/include)
add_executable(detect_probe ${CMAKE_SOURCE_DIR}/src/detect_probe.cpp)
target_include_directories(detect_probe PRIVATE
  ${CMAKE_SOURCE_DIR}/include ${HIK_INCLUDE})
target_link_libraries(detect_probe PRIVATE
  ${OpenCV_LIBS} ${HIK_LINK_LIB} Threads::Threads)
set_target_properties(detect_probe PROPERTIES
  BUILD_RPATH "${HIK_RPATH}" INSTALL_RPATH "${HIK_RPATH}")
