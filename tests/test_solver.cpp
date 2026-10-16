// test_solver.cpp: PnP 解算精度测试 (用合成靶标, 不需要相机)
#include "synth.hpp"
#include "test_util.hpp"

using namespace srm;
using namespace srm::test;

int main() {
    // ---------------------------------------------------------- 正对光心 5 m
    {
        SynthPose pose;
        pose.z_mm = 5000.0f;
        const auto s = make_armor_image(pose);
        CHECK(s.ok);
        const SolveResult r = solve_pnp(s.corners);
        CHECK(r.ok);
        CHECK_NEAR(r.dist, 5.0f, 0.02f);
        CHECK_NEAR(r.yaw, 0.0f, 0.2f);
        CHECK_NEAR(r.pitch, 0.0f, 0.2f);
    }

    // ---------------------------------------------------------- 距离扫描
    for (const float z : {1000.0f, 2000.0f, 3000.0f, 5000.0f, 7000.0f}) {
        SynthPose pose;
        pose.z_mm = z;
        const auto s = make_armor_image(pose);
        const SolveResult r = solve_pnp(s.corners);
        CHECK(r.ok);
        CHECK_NEAR(r.dist, z / 1000.0f, 0.02f);
    }

    // ---------------------------------------------------------- 横向/纵向偏移 -> 角度符号
    {
        // 目标在光轴右侧: 相机系 X 为正
        SynthPose pose;
        pose.x_mm = 500.0f;
        pose.z_mm = 5000.0f;
        auto s = make_armor_image(pose);
        SolveResult r = solve_pnp(s.corners);
        CHECK(r.ok);
        CHECK(r.yaw > 0.0f);                       // 偏右 -> yaw 正
        CHECK_NEAR(r.yaw, std::atan2(500.0f, 5000.0f) * 180.0 / CV_PI, 0.5);

        // 目标在光轴下方: 相机系 Y 为正 -> pitch 正
        SynthPose pose2;
        pose2.y_mm = 400.0f;
        pose2.z_mm = 5000.0f;
        s = make_armor_image(pose2);
        r = solve_pnp(s.corners);
        CHECK(r.ok);
        CHECK(r.pitch > 0.0f);
        CHECK_NEAR(r.pitch, std::atan2(400.0f, 5000.0f) * 180.0 / CV_PI, 0.5);
    }

    // ---------------------------------------------------------- 目标绕竖直轴旋转
    // PnP 只能看到平面, 大角度斜视时角度会偏, 但要保证解算不崩且距离仍合理
    for (const float yaw : {0.0f, 15.0f, 30.0f, 45.0f}) {
        SynthPose pose;
        pose.z_mm = 3000.0f;
        pose.yaw_deg = yaw;
        const auto s = make_armor_image(pose);
        const SolveResult r = solve_pnp(s.corners);
        CHECK(r.ok);
        CHECK(r.dist > 2.0f && r.dist < 5.0f);
    }

    // ---------------------------------------------------------- 无效输入
    {
        CHECK(!solve_pnp({}).ok);
        CHECK(!solve_pnp({cv::Point2f(0, 0)}).ok);
        std::vector<cv::Point2f> bad{cv::Point2f(1, 1), cv::Point2f(2, 2),
                                     cv::Point2f(3, 3), cv::Point2f(4, 4)};
        CHECK(!solve_pnp(bad).ok);   // 退化(共线)必须返回失败而不是瞎给结果
    }

    // ---------------------------------------------------------- 弹道补偿
    {
        // 5 m / 22 m/s: t=0.227s, drop=0.253m -> atan(0.253/5)=2.90 度
        CHECK_NEAR(ballistic_pitch_deg(5.0f, 22.0f), 2.90f, 0.05f);
        CHECK_NEAR(ballistic_pitch_deg(0.0f, 22.0f), 0.0f, 1e-6);
        // 弹速非法时用默认值兜底
        CHECK_NEAR(ballistic_pitch_deg(5.0f, 0.0f), ballistic_pitch_deg(5.0f, 22.0f), 1e-6);
        // 距离越远补偿越大
        CHECK(ballistic_pitch_deg(8.0f, 22.0f) > ballistic_pitch_deg(3.0f, 22.0f));
    }

    TEST_MAIN_END();
}
