// test_fire.cpp: 开火判据的离线验证
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "fire.hpp"

static int fails = 0;
#define CHECK(c, m) do { if(!(c)){ std::printf("  [FAIL] %s (%d)\n", m, __LINE__); ++fails; } \
                        else { std::printf("  [ ok ] %s\n", m); } } while(0)

using namespace srm;

int main() {
    std::printf("=== 1. 容差随距离缩放 ===\n");
    for (float d : {2.0f, 5.0f, 8.0f}) {
        std::printf("  %.0fm: 板角宽 %.2f 度, 容差 %.2f 度\n",
                    (double)d, (double)plate_angular_width_deg(d),
                    (double)fire_tolerance_deg(d));
    }
    CHECK(fire_tolerance_deg(2.0f) > fire_tolerance_deg(5.0f), "近处容差大于远处");
    CHECK(fire_tolerance_deg(5.0f) > fire_tolerance_deg(8.0f), "5m 容差大于 8m");
    // 5m 处板角宽约 1.6 度, 容差应约 0.96
    // 用公式推导期望值, 而不是写死某个数字 —— 参数调整('FIRE_TOL_FRAC')时
    // 这条测试应当继续有效, 它验证的是"容差 = 系数 × 板角宽"这个**关系**。
    const float expect5 = FIRE_TOL_FRAC * plate_angular_width_deg(5.0f);
    CHECK(std::fabs(fire_tolerance_deg(5.0f) - expect5) < 1e-4f, "5m 容差 = FIRE_TOL_FRAC × 板角宽");

    std::printf("\n=== 2. 同一个角度误差在不同距离的判定 ===\n");
    const bool ok2m = fire_decision(true, true, true, true, true, 2.0f, 1.2f, 0.0f, 22.0f, 10);
    const float tol5 = fire_tolerance_deg(5.0f);
    const bool ok5m = fire_decision(true, true, true, true, true, 5.0f, tol5 * 1.3f, 0.0f, 22.0f, 10);
    std::printf("  5m 容差 %.2f 度, 用 %.2f 度测试: 2m=%d, 5m=%d\n",
                (double)tol5, (double)(tol5 * 1.3f), ok2m ? 1 : 0, ok5m ? 1 : 0);
    CHECK(ok2m, "2m 处同样角度可以开火(容差随距离放大)");
    CHECK(!ok5m, "超过 5m 容差 1.3 倍时不开火");

    std::printf("\n=== 3. 每个门控条件都能单独否决 ===\n");
    const bool base = fire_decision(true, true, true, true, true, 3.0f, 0.1f, 0.1f, 22.0f, 10);
    CHECK(base, "全部满足时可以开火");
    CHECK(!fire_decision(false,true,true,true,true,3.0f,0.1f,0.1f,22.0f,10), "未开启 -> 否");
    CHECK(!fire_decision(true,false,true,true,true,3.0f,0.1f,0.1f,22.0f,10), "未锁定 -> 否");
    CHECK(!fire_decision(true,true,false,true,true,3.0f,0.1f,0.1f,22.0f,10), "视觉未开 -> 否");
    CHECK(!fire_decision(true,true,true,false,true,3.0f,0.1f,0.1f,22.0f,10), "回传过期 -> 否");
    CHECK(!fire_decision(true,true,true,true,false,3.0f,0.1f,0.1f,22.0f,10), "无目标 -> 否");
    CHECK(!fire_decision(true,true,true,true,true,9.0f,0.1f,0.1f,22.0f,10), "超距离 -> 否");
    CHECK(!fire_decision(true,true,true,true,true,3.0f,0.1f,0.1f,0.0f,10), "弹速无效 -> 否");
    // 用 FIRE_LOCK_FRAMES-1 而不是写死 3: 验的是"不足就不开火"这个关系
    CHECK(!fire_decision(true,true,true,true,true,3.0f,0.1f,0.1f,22.0f, FIRE_LOCK_FRAMES - 1),
          "锁定帧数不足 -> 否");
    CHECK(fire_decision(true,true,true,true,true,3.0f,0.1f,0.1f,22.0f, FIRE_LOCK_FRAMES),
          "刚好够帧数 -> 可以");

    std::printf("\n=== 4. 容差边界 ===\n");
    const float tol3 = fire_tolerance_deg(3.0f);
    CHECK(fire_decision(true,true,true,true,true,3.0f,tol3*0.95f,0,22.0f,10), "容差内可开火");
    CHECK(!fire_decision(true,true,true,true,true,3.0f,tol3*1.05f,0,22.0f,10), "超容差不打");

    std::printf("\n%s (%d 个失败)\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}
