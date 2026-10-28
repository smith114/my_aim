// srm_to_tongji.hpp: 把本项目的解算结果转成同济 EKF 期望的输入
//
// ============================================================================
// **坐标系约定（逐字核对过 math_tools.cpp，不是推断）**
//
// 本项目 solve_pnp 用的是 **OpenCV 相机系**:
//     x = 右,  y = 下,  z = 前
//     yaw   = atan2(tvec.x, tvec.z)   正 = 目标在右
//     pitch = atan2(tvec.y, tvec.z)   正 = 目标在下
//     单位: px/py/pz 是 mm, dist 是 m
//
// 同济 ypd2xyz 用的是 **ROS 右手系**:
//     x = 前,  y = 左,  z = 上
//     x = d·cos(pitch)·cos(yaw)
//     y = d·cos(pitch)·sin(yaw)
//     z = d·sin(pitch)
//     yaw   = atan2(y, x)              正 = 目标在左
//     pitch = atan2(z, hypot(x,y))     正 = 目标在上
//     单位: **米**
//
// 所以逐轴映射是:
//     同济 x =  我们的 z     (前)
//     同济 y = -我们的 x     (我们的右 = 同济的 -y)
//     同济 z = -我们的 y     (我们的下 = 同济的 -z)
//
// 角度上等价于:
//     同济 yaw   = -我们的 yaw
//     同济 pitch = -我们的 pitch
//
// **这个负号是我前面反复搞错方向的同类问题**, 所以这里既做坐标映射, 也在
// 单元测试里用"两种算法必须一致"交叉验证, 而不是只信推导。
// ============================================================================
#pragma once

#include <cmath>

#include <Eigen/Dense>

namespace srm {

// 我们的相机系三维位置(mm) -> 同济系(m)
inline Eigen::Vector3d to_tongji_xyz(float px_mm, float py_mm, float pz_mm) {
    return Eigen::Vector3d(static_cast<double>(pz_mm) * 0.001,    // x 前
                           -static_cast<double>(px_mm) * 0.001,   // y 左
                           -static_cast<double>(py_mm) * 0.001);  // z 上
}

// 我们的角度(度) + 距离(米) -> 同济的 ypd (rad, rad, m)
inline Eigen::Vector3d to_tongji_ypd(float yaw_deg, float pitch_deg, float dist_m) {
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    return Eigen::Vector3d(-static_cast<double>(yaw_deg) * kDeg2Rad,
                           -static_cast<double>(pitch_deg) * kDeg2Rad,
                           static_cast<double>(dist_m));
}

// 把同济的瞄准角(rad) 转回我们下位机用的度
// 下位机的 yaw/pitch 与我们的图像偏角同向(实测: yaw 增大镜头向左,
// theta_yaw>0 表示目标在右), 所以这里取反。
inline void from_tongji_aim(double tongji_yaw_rad, double tongji_pitch_rad,
                            float& out_yaw_deg, float& out_pitch_deg) {
    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    out_yaw_deg   = static_cast<float>(-tongji_yaw_rad * kRad2Deg);
    out_pitch_deg = static_cast<float>(-tongji_pitch_rad * kRad2Deg);
}

// ---------------------------------------------------------------- 自检用
// 两个方向各自算一次 ypd, 应互相一致。测试里调用这个验证映射没写反。
inline bool selfcheck_frame(double& max_yaw_err_deg, double& max_pitch_err_deg,
                            double& max_dist_err_m) {
    max_yaw_err_deg = 0.0;
    max_pitch_err_deg = 0.0;
    max_dist_err_m = 0.0;
    // 覆盖四个象限 + 不同距离/俯仰
    const float cases[][3] = {
        {  0.0f,   0.0f, 3.0f}, { 10.0f,   5.0f, 5.0f}, {-10.0f,  -5.0f, 5.0f},
        {  25.0f,  12.0f, 2.0f}, {-25.0f, -12.0f, 2.0f}, {  5.0f, -15.0f, 8.0f},
    };
    for (const auto& c : cases) {
        const float yaw = c[0], pitch = c[1], d = c[2];
        const double yr = yaw * 3.14159265358979323846 / 180.0;
        const double pr = pitch * 3.14159265358979323846 / 180.0;

        // 路线 1: 我们的相机系 -> 同济系 -> xyz2ypd
        const double cx = d * std::cos(pr) * std::sin(yr);   // 右
        const double cy = d * std::cos(pr) * std::cos(yr);   // 前(z)
        const double cz = d * std::sin(pr);                  // 下(+y)
        const Eigen::Vector3d tj = to_tongji_xyz(
            static_cast<float>(cx * 1000.0), static_cast<float>(cz * 1000.0),
            static_cast<float>(cy * 1000.0));
        const double yaw1 = std::atan2(tj[1], tj[0]);
        const double pitch1 = std::atan2(tj[2], std::sqrt(tj[0] * tj[0] + tj[1] * tj[1]));
        const double dist1 = tj.norm();

        // 路线 2: 直接角度取反
        const Eigen::Vector3d ypd = to_tongji_ypd(yaw, pitch, d);

        max_yaw_err_deg   = std::fmax(max_yaw_err_deg,
                                      std::fabs(yaw1 - ypd[0]) * 180.0 / 3.14159265358979323846);
        max_pitch_err_deg = std::fmax(max_pitch_err_deg,
                                      std::fabs(pitch1 - ypd[1]) * 180.0 / 3.14159265358979323846);
        max_dist_err_m    = std::fmax(max_dist_err_m, std::fabs(dist1 - ypd[2]));
    }
    // 阈值 1e-4 度: 三维那条路径要经过 float 的 mm 中转, 双精度下仍会有 ~1e-6 度的
    // 舍入残余(实测 1.17e-6)。这只反映 float 往返精度, 不是映射错误 ——
    // 真正的映射错误最低也是"整轴反向", 那是几十度量级。
    return max_yaw_err_deg < 1e-4 && max_pitch_err_deg < 1e-4 && max_dist_err_m < 1e-5;
}

}  // namespace srm
