// solver.hpp: PnP 解算 + 相机系偏差角 + 弹道补偿
//
// 坐标约定 (右手系, 物点单位 mm / 输出单位 m):
//   相机坐标系 (OpenCV 标准): +X 图像右, +Y 图像下, +Z 沿光轴向前
//   目标方向单位向量 d = tvec / |tvec|
//     d.yaw_cam   = atan2(d.x, d.z)   > 0 : 目标在光轴右侧
//     d.pitch_cam = atan2(d.y, d.z)   > 0 : 目标在光轴下方
//   PnP 的物点用 mm, 因此 tvec 也是 mm, 所有长度输出前统一 /1000 转米
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.hpp"

namespace srm {

// 相机系下的一次解算结果
struct SolveResult {
    // 目标中心在相机系的三维位置(mm) —— 给 3D 卡尔曼用
    float px = 0.0f, py = 0.0f, pz = 0.0f;
    bool  ok    = false;
    float dist  = 0.0f;   // 目标到相机光心距离, 米
    float yaw   = 0.0f;   // 相机系 yaw 偏差角, 度 (+ = 目标偏右)
    float pitch = 0.0f;   // 相机系 pitch 偏差角, 度 (+ = 目标偏下)
};

inline cv::Mat camera_matrix() {
    return (cv::Mat_<double>(3, 3) << CAM_FX, 0, CAM_CX,
                                      0, CAM_FY, CAM_CY,
                                      0, 0, 1);
}

inline cv::Mat distortion_coeffs() {
    return (cv::Mat_<double>(5, 1) << CAM_K1, CAM_K2, CAM_P1, CAM_P2, CAM_K3);
}

// 装甲板物点: 左上 / 右上 / 右下 / 左下 (与 armor.hpp 角点顺序一致)
// 平面法向 +Z 指向装甲板正面(朝相机)
inline std::vector<cv::Point3f> armor_object_points() {
    const float hw = ARMOR_W * 0.5f;
    const float hh = ARMOR_H * 0.5f;
    return {cv::Point3f(-hw, -hh, 0.0f),
            cv::Point3f( hw, -hh, 0.0f),
            cv::Point3f( hw,  hh, 0.0f),
            cv::Point3f(-hw,  hh, 0.0f)};
}

// 相机系位姿 (rvec/tvec, tvec 单位 mm)
struct PoseResult {
    bool    ok = false;
    cv::Mat rvec, tvec;
};

// 四点是否构成一个"像个四边形"的形状。
// 判据: 面积不能太小, 四条边不能太短, 且必须凸(任意相邻边叉积同号)。
// 这里只拦"明显退化"的输入; 目标在画面里很小(比如 4x4 px)时 PnP 精度本来就差,
// 那种情况交给 SOLVE_DIST_MAX / 角度上限等业务规则去过滤, 不在这里一刀切。
inline bool is_valid_quad(const std::vector<cv::Point2f>& p) {
    if (p.size() != 4) return false;
    // 鞋带公式算四边形面积
    double area2 = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f& a = p[i];
        const cv::Point2f& b = p[(i + 1) % 4];
        area2 += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    if (std::fabs(area2) * 0.5 < 4.0) return false;   // 面积 < 4 px^2 判为退化

    // 相邻边叉积方向应一致(凸性), 且每条边不能为零长
    double prev_sign = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f e1 = p[(i + 1) % 4] - p[i];
        const cv::Point2f e2 = p[(i + 2) % 4] - p[(i + 1) % 4];
        if (cv::norm(e1) < 1.0 || cv::norm(e2) < 1.0) return false;
        const double cross = static_cast<double>(e1.x) * e2.y - static_cast<double>(e1.y) * e2.x;
        if (std::fabs(cross) < 1e-6) return false;     // 共线
        if (prev_sign != 0.0 && ((cross > 0) != (prev_sign > 0))) return false;  // 非凸
        prev_sign = cross;
    }
    return true;
}

inline PoseResult solve_pose(const std::vector<cv::Point2f>& img_pts) {
    PoseResult pr;
    if (img_pts.size() != 4) return pr;
    // 退化检查: 四点近共线 / 近似重合时 PnP 会给出完全错误的位姿, 必须提前拦掉
    if (!is_valid_quad(img_pts)) return pr;
    const auto obj = armor_object_points();
    // 四个点共面, IPPE 专为该情形设计, 比 ITERATIVE 稳且不需初值
    pr.ok = cv::solvePnP(obj, img_pts, camera_matrix(), distortion_coeffs(),
                         pr.rvec, pr.tvec, false, cv::SOLVEPNP_IPPE);
    if (!pr.ok) {   // 退化时退回迭代法
        pr.ok = cv::solvePnP(obj, img_pts, camera_matrix(), distortion_coeffs(),
                             pr.rvec, pr.tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    return pr;
}

// 给四个像素角点(左上/右上/右下/左下), 解出距离和相机系偏差角
inline SolveResult solve_pnp(const std::vector<cv::Point2f>& img_pts) {
    SolveResult out;
    const PoseResult pr = solve_pose(img_pts);
    if (!pr.ok) return out;

    const double x = pr.tvec.at<double>(0);
    const double y = pr.tvec.at<double>(1);
    const double z = pr.tvec.at<double>(2);
    if (z <= 1.0) return out;   // 目标在光轴后方或贴脸, 判为无效

    out.px    = static_cast<float>(x);
    out.py    = static_cast<float>(y);
    out.pz    = static_cast<float>(z);
    out.dist  = static_cast<float>(std::sqrt(x * x + y * y + z * z) / 1000.0);
    out.yaw   = static_cast<float>(std::atan2(x, z) * 180.0 / CV_PI);
    out.pitch = static_cast<float>(std::atan2(y, z) * 180.0 / CV_PI);
    out.ok    = true;
    return out;
}

// ------------------------------------------------------------- 弹道补偿
// 小弹丸近似: 只考虑重力, 忽略空气阻力。
//   t    = dist / v
//   drop = 0.5 * g * t^2                 (m)
//   pitch_comp = atan2(drop, dist)       (度)
// 含义: 为了打到目标, 枪口需要比视线再抬高多少。
// 旋转靶标中心高度基本不变, 所以此处只补重力; 横向提前量需要目标角速度,
// 留给终验阶段的预测器。
inline float ballistic_pitch_deg(float dist_m, float bullet_speed_mps) {
    if (dist_m <= 0.0f) return 0.0f;
    if (!(bullet_speed_mps > 1.0f)) bullet_speed_mps = BULLET_SPEED_DEFAULT;
    const float t    = dist_m / bullet_speed_mps;
    const float drop = 0.5f * BALLISTIC_G * t * t;
    return static_cast<float>(std::atan2(drop, dist_m) * 180.0 / CV_PI);
}

}  // namespace srm
