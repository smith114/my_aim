// test_frame.cpp: 验证 本相机系 <-> 同济 ROS 系 的映射没写反
//
// 这里不用"推导正确所以应该对", 而是用两条独立路径互相校验:
//   路径1: 三维坐标逐轴映射 -> atan2 反算角度
//   路径2: 直接对角度取反
// 两者必须一致到 1e-6 度, 否则映射有错。
#include <cstdio>

#include "srm_to_tongji.hpp"

int main() {
    int fails = 0;
    double ey = 0, ep = 0, ed = 0;
    const bool ok = srm::selfcheck_frame(ey, ep, ed);
    std::printf("两条独立路径的最大偏差: yaw %.3e 度, pitch %.3e 度, dist %.3e m\n",
                ey, ep, ed);
    if (!ok) { std::printf("  [FAIL] 坐标映射不一致\n"); ++fails; }
    else     { std::printf("  [ ok ] 坐标映射一致\n"); }

    // 显式断言符号: 目标在右侧(theta_yaw>0) -> 同济 yaw 必须为负(y 向左)
    float y = 0, p = 0;
    const auto ypd = srm::to_tongji_ypd(10.0f, 5.0f, 5.0f);
    std::printf("本系 yaw=+10 pitch=+5 -> 同济 yaw=%+.2f 度 pitch=%+.2f 度\n",
                ypd[0] * 57.29578, ypd[1] * 57.29578);
    if (!(ypd[0] < 0.0)) { std::printf("  [FAIL] 同济 yaw 应为负\n"); ++fails; }
    else std::printf("  [ ok ] 同济 yaw 为负(本系正 yaw = 目标在右 = 同济 -y)\n");
    if (!(ypd[1] < 0.0)) { std::printf("  [FAIL] 同济 pitch 应为负\n"); ++fails; }
    else std::printf("  [ ok ] 同济 pitch 为负(本系正 pitch = 目标在下 = 同济 -z)\n");

    // 往返: 同济瞄准角 -> 本系角度
    srm::from_tongji_aim(ypd[0], ypd[1], y, p);
    std::printf("往返: 同济 -> 本系 yaw=%+.2f pitch=%+.2f (应回到 +10/+5)\n", y, p);
    if (std::fabs(y - 10.0f) > 1e-4f || std::fabs(p - 5.0f) > 1e-4f) {
        std::printf("  [FAIL] 往返不一致\n"); ++fails;
    } else std::printf("  [ ok ] 往返一致\n");

    std::printf("\n%s (%d 个失败)\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}
