// test_util.hpp: 极简测试宏 (不引入 gtest 依赖, 直接 ctest 跑)
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

namespace srm {
namespace test {

inline int& fail_count() {
    static int n = 0;
    return n;
}
inline int& check_count() {
    static int n = 0;
    return n;
}

inline void report(const char* file) {
    std::printf("\n[%s] 检查 %d 项, 失败 %d 项 -> %s\n", file, check_count(), fail_count(),
                fail_count() == 0 ? "PASS" : "FAIL");
}

}  // namespace test
}  // namespace srm

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++::srm::test::check_count();                                            \
        if (!(cond)) {                                                           \
            ++::srm::test::fail_count();                                         \
            std::printf("  [FAIL] %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                         \
        ++::srm::test::check_count();                                            \
        const double _va = (a), _vb = (b), _vt = (tol);                          \
        if (!(std::fabs(_va - _vb) <= _vt)) {                                    \
            ++::srm::test::fail_count();                                         \
            std::printf("  [FAIL] %s:%d  %s(%.4f) != %s(%.4f) tol=%.4f\n",       \
                        __FILE__, __LINE__, #a, _va, #b, _vb, _vt);              \
        }                                                                        \
    } while (0)

#define TEST_MAIN_END()                                                          \
    do {                                                                         \
        ::srm::test::report(__FILE__);                                           \
        return ::srm::test::fail_count() == 0 ? 0 : 1;                           \
    } while (0)
