// gimbal_test.cpp: 云台诊断工具 —— 不用手推云台, 全靠软件测
//
// 用法:
//   ./gimbal_test --query                      只读: 打印当前实测姿态后退出
//   ./gimbal_test --hold                       只发"保持当前角", 看云台动不动
//   ./gimbal_test --step yaw 2                 朝 +yaw 走一步 2 度(可多次执行)
//   ./gimbal_test --sweep yaw + --stop 8       向 +yaw 扫描, 直到实测角连续
//                                              8 次不再变化(说明顶到限位了)就停
//
// 安全性:
//   * 只发送"当前实测角 + 小步长", 不锚在会累积的估计值上;
//   * 每一步都等实测角稳定再走下一步, 不会连发;
//   * 实测角长时间不更新(>0.3s)或串口断开, 立即停止并补发保持帧;
//   * 默认单次扫描上限 --max-deg 200 度, 防止失控一路转;
//   * Ctrl-C 立即停。
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "config.hpp"
#include "protocol.hpp"

using namespace srm;

static std::atomic<bool> g_stop{false};
static void on_sig(int) { g_stop = true; }

static void usage() {
    std::printf(
        "用法:\n"
        "  gimbal_test --query                 只读当前实测姿态\n"
        "  gimbal_test --stay                  锁住当前姿态不归零(手动对准后停在这)\n"
        "  gimbal_test --watch                 实时盯 mode/姿态 (诊断鼠标右键是否生效)\n"
        "  gimbal_test --hold                  保持当前角(不接受目标角参数!)\n"
                         "  gimbal_test --step yaw 30           在当前角基础上走 +30 度\n"
        "  gimbal_test --step <yaw|pitch> <deg> [--frames N]\n"
        "  gimbal_test --sweep <+|-><yaw|pitch> [--stop N] [--max-deg D] [--port P]\n"
        "例子:\n"
        "  ./gimbal_test --query\n"
        "  ./gimbal_test --watch\n"
        "  ./gimbal_test --sweep +yaw --stop 8\n"
        "  ./gimbal_test --sweep -pitch --stop 8 --max-deg 90\n");
}

int main(int argc, char** argv) {
    std::string mode;
    std::string axis;
    std::string sweep_dir;
    float  step_deg   = 2.0f;
    int    stop_n     = 8;
    float  max_deg    = 200.0f;
    int    frames     = 60;
    std::string port_override;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--query") {
            mode = "query";
        } else if (a == "--hold") {
            mode = "hold";
        } else if (a == "--watch") {
            mode = "watch";
        } else if (a == "--stay") {
            mode = "stay";
        } else if (a == "--step" && i + 2 < argc) {
            mode = "step";
            axis = argv[++i];
            step_deg = std::atof(argv[++i]);
        } else if (a == "--sweep" && i + 1 < argc) {
            mode = "sweep";
            const std::string d = argv[++i];
            if (!d.empty()) {
                sweep_dir = d.substr(0, 1);
                axis = d.substr(1);
            }
        } else if (a == "--stop" && i + 1 < argc) {
            stop_n = std::atoi(argv[++i]);
        } else if (a == "--max-deg" && i + 1 < argc) {
            max_deg = std::atof(argv[++i]);
        } else if (a == "--frames" && i + 1 < argc) {
            frames = std::atoi(argv[++i]);
        } else if (a == "--port" && i + 1 < argc) {
            port_override = argv[++i];
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        }
    }
    if (mode.empty()) {
        usage();
        return 2;
    }

    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    // ---------------------------------------------------------- 打开串口
    std::string port_name;
    int fd = -1;
    if (!port_override.empty()) {
        fd = open_port(port_override);
        port_name = port_override;
    } else {
        fd = open_first_available({SERIAL_BY_ID, SERIAL_FALLBACK0, SERIAL_FALLBACK1},
                                  &port_name);
    }
    if (fd < 0) {
        std::fprintf(stderr, "[ERR] 打不开下位机串口, 请检查设备名/权限\n");
        return 1;
    }
    std::printf("[SERIAL] %s\n", port_name.c_str());

    SerialLink link;
    link.start(fd, SERIAL_SEND_HZ, SERIAL_WATCHDOG_MS, SERIAL_SHUTDOWN_ZERO_FRAMES);

    // ---------------------------------------------------------- 等首帧
    RxState st;
    for (int i = 0; i < 300 && !g_stop; ++i) {
        st = link.state();
        if (st.valid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!st.valid) {
        std::fprintf(stderr, "[ERR] 没收到下位机回传(3s), 检查接线与设备名\n");
        link.stop();
        return 1;
    }
    std::printf("[START] 实测姿态 yaw=%.2f pitch=%.2f roll=%.2f bullet=%.2f\n",
                st.yaw, st.pitch, st.roll, st.bullet_speed);
    std::printf("        角度是连续值, 记录下列数值即可知道实际量程与方向\n\n");

    // 初始就一直发当前实测角, 让云台先停住
    link.set_target(st.yaw, st.pitch, 0);

    // ---------------------------------------------------------- --query
    if (mode == "query") {
        for (int i = 0; i < 20 && !g_stop; ++i) {
            st = link.state();
            link.set_target(st.yaw, st.pitch, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::printf("当前实测: yaw=%.2f pitch=%.2f roll=%.2f (mode=%d id=%d)\n",
                    st.yaw, st.pitch, st.roll, st.mode, st.robot_id);
        std::printf("rx=%llu bad=%llu resync=%llu\n",
                    (unsigned long long)link.rx_frames(),
                    (unsigned long long)link.rx_bad_frames(),
                    (unsigned long long)link.resync_drops());
        link.stop();
        return 0;
    }

    // ---------------------------------------------------------- --watch
    // 实时盯 mode 与姿态变化。用来判断"鼠标右键到底有没有生效":
    // 按一次右键, 如果下位机真的收到并切换了视觉模式, 这里的 mode 会跳变并打横幅。
    // 如果 mode 一直不动, 说明右键没到下位机(鼠标/接线/固件侧问题), 与上位机无关。
    if (mode == "watch") {
        std::printf("实时监视中 (Ctrl-C 退出)。请现在去按鼠标右键进入视觉控制模式。\n");
        std::printf("判据: mode 由 0 跳到 1 = 下位机已进入视觉模式。\n\n");
        std::printf("   时间     yaw      pitch    roll     mode  id  帧数   说明\n");
        int last_mode = -999;
        const auto t0 = std::chrono::steady_clock::now();
        while (!g_stop) {
            st = link.state();
            // 一直发"保持当前角", 这样云台不会因为我们不动而漂走
            if (st.valid) link.set_target(st.yaw, st.pitch, 0);

            if (st.mode != last_mode) {
                if (last_mode != -999) {
                    std::printf("\n  ================= mode %d -> %d =================\n",
                                last_mode, st.mode);
                    if (st.mode == 1) {
                        std::printf("  >>> 已进入视觉控制模式, 下位机现在会接受上位机的角度指令 <<<\n\n");
                    } else {
                        std::printf("  >>> 已退出视觉控制模式, 下位机将忽略角度指令 <<<\n\n");
                    }
                }
                last_mode = st.mode;
            }

            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0).count();
            std::printf("\r%6.1fs  %8.2f %8.2f %7.2f  %4d  %3d  %5llu",
                        ms / 1000.0, st.yaw, st.pitch, st.roll, st.mode, st.robot_id,
                        (unsigned long long)link.rx_frames());
            std::fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        std::printf("\n");
        link.stop();
        return 0;
    }

    // ---------------------------------------------------------- --stay
    // 锁住当前姿态: 一直把"当前实测角"当命令发回去, 直到 Ctrl-C。
    //
    // 这就是"手动对好之后停住别动"的用法:
    //   * 每帧重新读实测角再原样发回 -> 你用手推它, 它会跟着走, 不跟你顶牛;
    //   * 松手后它会停在你放的位置(实测角不再变, 命令也就不再变);
    //   * 退出时补发的也是这个角(见 SerialLink::stop), **不会回到 0/0**;
    //   * 程序启动时不发任何帧, 直到读到第一帧回传(见 SerialLink::start)。
    if (mode == "stay") {
        std::printf("已锁住当前姿态, Ctrl-C 退出。程序不会把它拉回 0/0。\n");
        std::printf("(你现在可以用手把云台推到想要的角度, 松手后它会停在那儿)\n\n");
        float last_reported_y = 1e9f, last_reported_p = 1e9f;
        int  held = 0;
        while (!g_stop) {
            st = link.state();
            if (st.valid) {
                link.set_target(st.yaw, st.pitch, 0);   // 原样发回 = 锁住
                ++held;
                // 只在有人动了它或每隔 2 秒打一次, 避免刷屏
                const bool moved = std::fabs(st.yaw - last_reported_y) > 0.3f ||
                                   std::fabs(st.pitch - last_reported_p) > 0.3f;
                static int tick = 0;
                if (moved || (++tick % 20 == 0)) {
                    std::printf("已锁定 yaw=%.2f pitch=%.2f (mode=%d) 已保持 %d 帧\n",
                                st.yaw, st.pitch, st.mode, held);
                    std::fflush(stdout);
                    last_reported_y = st.yaw;
                    last_reported_p = st.pitch;
                }
            } else {
                std::printf("等回传...\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::printf("\n退出, 最后锁定在 yaw=%.2f pitch=%.2f\n", st.yaw, st.pitch);
        link.stop();
        return 0;
    }

    // ---------------------------------------------------------- --hold
    if (mode == "hold") {
        std::printf("保持当前角 %d 帧, 观察云台是否静止(若缓慢漂移说明有零偏)\n", frames);
        std::printf("  frame   cmd(yaw/pitch)      fb(yaw/pitch)\n");
        for (int i = 0; i < frames && !g_stop; ++i) {
            st = link.state();
            link.set_target(st.yaw, st.pitch, 0);
            if (i % 10 == 0) {
                std::printf("  %5d   %8.2f/%8.2f   %8.2f/%8.2f\n", i, st.yaw, st.pitch,
                            st.yaw, st.pitch);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        link.stop();
        return 0;
    }

    // ---------------------------------------------------------- --step
    if (mode == "step") {
        if (axis != "yaw" && axis != "pitch") {
            std::fprintf(stderr, "[ERR] --step 的轴必须是 yaw 或 pitch\n");
            link.stop();
            return 2;
        }
        const float start_y = st.yaw;
        const float start_p = st.pitch;
        const float ty = (axis == "yaw")   ? st.yaw + step_deg : st.yaw;
        const float tp = (axis == "pitch") ? st.pitch + step_deg : st.pitch;
        std::printf("命令 %s %+.1f 度: %.2f/%.2f -> %.2f/%.2f\n", axis.c_str(), step_deg,
                    st.yaw, st.pitch, ty, tp);
        // 0.6 秒对上位机下发+下位机执行来说太短: 大步长还没到位就打印了结论,
        // 会把"还在路上"误判成"反向"。改成 2.5 秒。
        for (int i = 0; i < 125 && !g_stop; ++i) {
            link.set_target(ty, tp, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        st = link.state();
        std::printf("结果实测: yaw=%.2f pitch=%.2f  (期望 yaw=%.2f pitch=%.2f)\n",
                    st.yaw, st.pitch, ty, tp);
        // 判定只看"朝命令方向移动了没有", 不用窄窗口:
        // 用 (测量-起点)·(命令-起点) 的符号即可。过冲/欠冲都不影响方向判断。
        {
            const float d_meas = (axis == "yaw") ? (st.yaw - start_y) : (st.pitch - start_p);
            const float d_cmd  = (axis == "yaw") ? (ty - start_y)     : (tp - start_p);
            const float dot    = d_meas * d_cmd;
            std::printf("结论: 实测移动 %+.2f 度, 命令移动 %+.2f 度 => %s\n",
                        d_meas, d_cmd,
                        (std::fabs(d_meas) < 0.5f) ? "没动(通道异常)"
                        : (dot > 0 ? "同向(通道正常)" : "反向(命令符号与实测相反)"));
        }
        link.stop();
        return 0;
    }

    // ---------------------------------------------------------- --sweep
    if (mode == "sweep") {
        if (axis != "yaw" && axis != "pitch") {
            std::fprintf(stderr, "[ERR] --sweep 的轴必须是 yaw 或 pitch, 例如 +yaw\n");
            link.stop();
            return 2;
        }
        if (sweep_dir != "+" && sweep_dir != "-") {
            std::fprintf(stderr, "[ERR] --sweep 的方向必须是 + 或 -, 例如 +yaw\n");
            link.stop();
            return 2;
        }
        const float sgn = (sweep_dir == "+") ? 1.0f : -1.0f;
        const float step = 1.0f;   // 每步 1 度, 慢一点便于观察

        std::printf("向 %s%s 扫描: 每步 %.1f 度, 实测角连续 %d 步不变就判为顶到限位\n",
                    sweep_dir.c_str(), axis.c_str(), step, stop_n);
        std::printf("上限 %.0f 度; 随时 Ctrl-C 停止\n\n", max_deg);
        std::printf("  step   cmd(yaw/pitch)      fb(yaw/pitch)    fb变化\n");

        float cmd_y = st.yaw, cmd_p = st.pitch;
        const float start_y = st.yaw, start_p = st.pitch;
        float last_fb = (axis == "yaw") ? st.yaw : st.pitch;
        int   no_move = 0;
        int   step_i = 0;
        bool  hit_limit = false;

        while (!g_stop) {
            const float travelled = std::fabs((axis == "yaw" ? cmd_y : cmd_p) -
                                              (axis == "yaw" ? start_y : start_p));
            if (travelled > max_deg) {
                std::printf("\n[STOP] 已达单次扫描上限 %.0f 度(不代表到限位)\n", max_deg);
                break;
            }
            if (axis == "yaw") cmd_y += sgn * step;
            else              cmd_p += sgn * step;
            ++step_i;

            link.set_target(cmd_y, cmd_p, 0);

            // 等这一段走完并稳定, 再读实测
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            st = link.state();
            const float fb = (axis == "yaw") ? st.yaw : st.pitch;
            const float dfb = fb - last_fb;

            if (step_i % 5 == 0 || std::fabs(dfb) > 5.0f) {
                std::printf("  %4d   %8.2f/%8.2f   %8.2f/%8.2f   %+7.2f\n", step_i, cmd_y,
                            cmd_p, st.yaw, st.pitch, dfb);
            }

            if (std::fabs(dfb) < 0.2f) {
                ++no_move;
                if (no_move >= stop_n) {
                    hit_limit = true;
                    break;
                }
            } else {
                no_move = 0;
            }
            last_fb = fb;
        }

        st = link.state();
        std::printf("\n===== 扫描结果 =====\n");
        std::printf("起点实测: yaw=%.2f pitch=%.2f\n", start_y, start_p);
        std::printf("终点实测: yaw=%.2f pitch=%.2f\n", st.yaw, st.pitch);
        if (axis == "yaw") {
            std::printf("实测 yaw 走过范围: %.2f 度\n", st.yaw - start_y);
        } else {
            std::printf("实测 pitch 走过范围: %.2f 度\n", st.pitch - start_p);
        }
        if (hit_limit) {
            std::printf("判定: 实测角已停止跟随命令, 该方向极限约 %s = %.2f 度\n",
                        axis.c_str(), (axis == "yaw") ? st.yaw : st.pitch);
        } else {
            std::printf("判定: 本次未探到极限(被上限或 Ctrl-C 中断)\n");
        }
        std::printf("命令通道: yaw 实际到 %.2f, pitch 实际到 %.2f\n", cmd_y, cmd_p);

        // 停在原地, 不要突然归零
        link.set_target(st.yaw, st.pitch, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        link.stop();
        return 0;
    }

    link.stop();
    return 0;
}
