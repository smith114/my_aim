// config.hpp: 所有可调参数集中在这里, 实车调参只改这个文件
#pragma once

// 相机内参 (demo.yaml, 实车需重新标定)
constexpr float CAM_FX = 1818.37f;
constexpr float CAM_FY = 1822.49f;
constexpr float CAM_CX = 751.06f;
constexpr float CAM_CY = 530.44f;

// 二值化阈值 (实车按光线调)
constexpr int BINARIZE_THRESH = 200;

// 灯条筛选
constexpr float LB_SHORT_MIN = 3.0f;    // 短边最小 px
constexpr float LB_LONG_MIN  = 8.0f;   // 长边最小 px
constexpr float LB_RATIO_MIN = 3.0f;   // 长宽比下限
constexpr float LB_RATIO_MAX = 15.0f; // 长宽比上限

// 装甲板配对约束
constexpr float PAIR_DIST_MIN   = 10.0f;
constexpr float PAIR_DIST_MAX   = 500.0f;
constexpr float PAIR_DY_MAX     = 30.0f;
constexpr float PAIR_LEN_RATIO  = 2.0f;
constexpr float PAIR_ANGLE_DIFF = 15.0f;
constexpr float ARMOR_RATIO_MIN = 0.3f;
constexpr float ARMOR_RATIO_MAX = 3.0f;

// 状态机
constexpr int MAX_LOST_FRAMES = 15;

// 装甲板物理尺寸 mm
constexpr float ARMOR_W = 140.0f;
constexpr float ARMOR_H = 125.0f;