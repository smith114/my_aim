// test_armor.cpp: 灯条提取 / 装甲板配对 / 检测到解算的端到端测试 (合成图)
#include "armor.hpp"
#include "solver.hpp"
#include "synth.hpp"
#include "test_util.hpp"

using namespace srm;
using namespace srm::test;

int main() {
    // ---------------------------------------------------------- 正对靶标能检出并解算
    {
        SynthPose pose;
        pose.z_mm = 5000.0f;
        const auto s = make_armor_image(pose);
        const auto bars = extract_lightbars(s.image);
        CHECK(bars.size() >= 2);   // 两块灯条

        cv::Mat hsv, ch[3];
        cv::cvtColor(s.image, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, ch);
        const auto armors = match_armors(bars, ch[2]);
        CHECK(!armors.empty());

        // 至少有一个配对能解出接近 5 m 的距离
        bool found = false;
        for (const auto& ar : armors) {
            const SolveResult r = solve_pnp(ar.pts);
            if (r.ok && std::fabs(r.dist - 5.0f) < 0.15f) found = true;
        }
        CHECK(found);
    }

    // ---------------------------------------------------------- 不同距离
    for (const float z : {1500.0f, 3000.0f, 5000.0f}) {
        SynthPose pose;
        pose.z_mm = z;
        const auto s = make_armor_image(pose);
        const auto bars = extract_lightbars(s.image);
        CHECK(bars.size() >= 2);
        cv::Mat hsv, ch[3];
        cv::cvtColor(s.image, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, ch);
        const auto armors = match_armors(bars, ch[2]);
        bool found = false;
        for (const auto& ar : armors) {
            const SolveResult r = solve_pnp(ar.pts);
            if (r.ok && std::fabs(r.dist - z / 1000.0f) < 0.2f) found = true;
        }
        CHECK(found);
    }

    // ---------------------------------------------------------- 红色靶标
    {
        SynthPose pose;
        pose.z_mm = 4000.0f;
        pose.color = 1;
        const auto s = make_armor_image(pose);
        const auto bars = extract_lightbars(s.image);
        CHECK(bars.size() >= 2);
        for (const auto& b : bars) CHECK(b.color == 1);
    }

    // ---------------------------------------------------------- 空图/纯噪声不产生装甲板
    {
        cv::Mat noise(1080, 1440, CV_8UC3);
        cv::RNG rng(7);
        rng.fill(noise, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(90, 90, 90));
        const auto bars = extract_lightbars(noise);
        cv::Mat hsv, ch[3];
        cv::cvtColor(noise, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, ch);
        const auto armors = match_armors(bars, ch[2]);
        CHECK(armors.empty());
    }

    // ---------------------------------------------------------- 角点排序正确性
    // 契约: 输出顺序必须是 左上(0) / 右上(1) / 右下(2) / 左下(3),
    // 与 armor_object_points() 的 (-hw,-hh)(hw,-hh)(hw,hh)(-hw,hh) 一一对应。
    // 图像 y 向下, 所以 -hh(物点 y 最小) 在图像上方。
    {
        // 故意打乱输入顺序, 且让两个"中间点"的 x+y 相等, 逼出排序的稳定性问题
        std::vector<cv::Point2f> pts{cv::Point2f(10, 100), cv::Point2f(100, 100),
                                     cv::Point2f(100, 10),  cv::Point2f(10, 10)};
        const auto s = sort_armor_points(pts);
        CHECK(s.size() == 4);
        CHECK_NEAR(s[0].x, 10.0f, 1e-3);    // 左上
        CHECK_NEAR(s[0].y, 10.0f, 1e-3);
        CHECK_NEAR(s[1].x, 100.0f, 1e-3);   // 右上
        CHECK_NEAR(s[1].y, 10.0f, 1e-3);
        CHECK_NEAR(s[2].x, 100.0f, 1e-3);   // 右下
        CHECK_NEAR(s[2].y, 100.0f, 1e-3);
        CHECK_NEAR(s[3].x, 10.0f, 1e-3);    // 左下
        CHECK_NEAR(s[3].y, 100.0f, 1e-3);
        // 排序结果必须是凸四边形, 否则 PnP 会给出错误位姿
        CHECK(is_valid_quad(s));
        // 反复调用结果必须一致(比较器不稳定会导致时对时错)
        for (int rep = 0; rep < 20; ++rep) {
            const auto s2 = sort_armor_points(pts);
            CHECK_NEAR(s2[1].x, 100.0f, 1e-3);
            CHECK_NEAR(s2[2].y, 100.0f, 1e-3);
        }
    }

    // ---------------------------------------------------------- 中心暗区校验
    {
        cv::Mat v(100, 100, CV_8UC1, cv::Scalar(200));
        CHECK(!armor_center_dark(v, cv::Point2f(50, 50), 10, 10));  // 亮 -> 不是真板
        v.setTo(cv::Scalar(50));
        CHECK(armor_center_dark(v, cv::Point2f(50, 50), 10, 10));   // 暗 -> 通过
        // 靠边太小无法判断 -> 放行
        CHECK(armor_center_dark(v, cv::Point2f(1, 1), 1, 1));
    }

    // ---------------------------------------------------------- 灯条方向角归一化
    {
        cv::RotatedRect rr(cv::Point2f(0, 0), cv::Size(10, 100), -80.0f);
        const float d = lightbar_dir(rr);
        CHECK(d >= 0.0f && d < 180.0f);
    }

    TEST_MAIN_END();
}
