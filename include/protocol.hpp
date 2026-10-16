// protocol.hpp: 校内赛自瞄通信协议 —— 组帧 / 流式解帧 / 收发线程
//
// 唯一依据: 《SRM 校内赛自瞄通信协议》
//   (control-2026 / infantry_leg&prostrate @ f7b83a58e8dc8af13ae5f328687c2fd7462e8b80)
//
//   body_len(int16 LE) + records(int16 id + 定长数据体)
//   上 -> 下  固定 18B : body_len=16, ID1 yaw/pitch(float32), ID2 fire_flag(int32)
//   下 -> 上  固定 30B : body_len=28, ID1 yaw/pitch/roll(f32) + mode/color(i32), ID2 bullet_speed(f32)
//
//   无魔数 / 无 CRC / 无时间戳 / 无序号 => 长度字段是唯一帧边界, 必须流式缓存组帧
//   USB 包边界 != 协议帧边界: 一次 read 可能拿到半帧、整帧或多帧
#pragma once

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace srm {

// ------------------------------------------------------------------ 协议常量
constexpr int16_t kBodyLenTx = 16;  // 上->下 records 字节数
constexpr int16_t kBodyLenRx = 28;  // 下->上 records 字节数
constexpr size_t  kFrameLenTx = 18; // 2 + 16
constexpr size_t  kFrameLenRx = 30; // 2 + 28
constexpr int16_t kIdGimbal  = 1;
constexpr int16_t kIdShoot   = 2;
// 裁判系统未注入时, 参考实现内部默认 22.0 m/s
constexpr float kDefaultBulletSpeed = 22.0f;

// ------------------------------------------------------------------ 小端读写
inline void put_i16(std::vector<uint8_t>& b, int16_t v) {
    b.push_back(static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xff));
    b.push_back(static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xff));
}
inline void put_i32(std::vector<uint8_t>& b, int32_t v) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
    b.insert(b.end(), p, p + 4);
}
inline void put_f32(std::vector<uint8_t>& b, float v) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
    b.insert(b.end(), p, p + 4);
}
inline int16_t get_i16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}
inline int32_t get_i32(const uint8_t* p) {
    uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                 (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(v);
}
inline float get_f32(const uint8_t* p) {
    uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                 (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float f = 0.0f;
    std::memcpy(&f, &v, 4);
    return f;
}

// 单调时钟(秒), 用于看门狗和延迟统计
inline double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ------------------------------------------------------------------ 下行状态
// 下位机一帧里给出的全部内容。缺某条记录时对应字段保持上一次的值。
struct RxState {
    bool    valid            = false;  // 是否收到过合法下行帧
    float   yaw              = 0.0f;   // 当前云台姿态, 度
    float   pitch            = 0.0f;
    float   roll             = 0.0f;
    int32_t mode             = 0;      // 参考固件固定发 0, 保留字段
    int32_t robot_id         = 0;      // 协议里字段名叫 color, 实际写机器人 ID
    float   bullet_speed     = kDefaultBulletSpeed;
    bool    has_bullet_speed = false;  // 是否真的收到过 ID2
    double  stamp            = 0.0;    // 本地接收时刻(秒)

    double age() const { return valid ? (now_s() - stamp) : 1e9; }
};

// ------------------------------------------------------------------ 组帧
// 上->下: 每帧必须同时包含 ID1 和 ID2, 顺序固定
inline std::vector<uint8_t> pack_tx(float yaw_deg, float pitch_deg, int32_t fire_flag) {
    std::vector<uint8_t> b;
    b.reserve(kFrameLenTx);
    put_i16(b, kBodyLenTx);   // body_len = 16
    put_i16(b, kIdGimbal);    // ID 1
    put_f32(b, yaw_deg);      //   Gimbal_Receive_s.yaw
    put_f32(b, pitch_deg);    //   Gimbal_Receive_s.pitch
    put_i16(b, kIdShoot);     // ID 2
    put_i32(b, fire_flag);    //   Shoot_Receive_s.fire_flag
    return b;
}

// ------------------------------------------------------------------ 流式解帧
// feed() 可被任意次调用, 每次喂入 read() 拿到的原始字节。
// 非法长度 / 未知 ID => 无法可靠重同步(协议无魔数无 CRC), 丢缓存等下一帧。
class RxParser {
public:
    // 返回本次接受的完整帧数
    int feed(const uint8_t* data, size_t n) {
        buf_.insert(buf_.end(), data, data + n);
        int frames = 0;
        while (true) {
            if (buf_.size() < 2) break;
            const int16_t len = get_i16(buf_.data());
            if (len != kBodyLenRx) {
                ++resync_drops_;
                buf_.clear();
                break;
            }
            if (buf_.size() < kFrameLenRx) break;   // 半帧, 继续等
            if (parse_records(buf_.data() + 2, static_cast<size_t>(len))) ++frames;
            buf_.erase(buf_.begin(), buf_.begin() + kFrameLenRx);
        }
        return frames;
    }

    size_t   pending() const { return buf_.size(); }
    uint64_t resync_drops() const { return resync_drops_; }
    uint64_t good_frames() const { return good_frames_; }
    uint64_t bad_frames() const { return bad_frames_; }

    RxState snapshot() const {
        std::lock_guard<std::mutex> lk(m_);
        return latest_;
    }

private:
    bool parse_records(const uint8_t* p, size_t len) {
        size_t off = 0;
        RxState s;
        bool got_any = false;
        while (off + 2 <= len) {
            const int16_t id = get_i16(p + off);
            off += 2;
            if (id == kIdGimbal) {
                if (off + 20 > len) break;
                s.yaw      = get_f32(p + off + 0);
                s.pitch    = get_f32(p + off + 4);
                s.roll     = get_f32(p + off + 8);
                s.mode     = get_i32(p + off + 12);
                s.robot_id = get_i32(p + off + 16);
                off += 20;
                got_any = true;
            } else if (id == kIdShoot) {
                if (off + 4 > len) break;
                s.bullet_speed     = get_f32(p + off);
                s.has_bullet_speed = true;
                off += 4;
                got_any = true;
            } else {
                ++bad_frames_;   // 未知 ID: 长度未知无法跳过
                return false;
            }
        }
        if (!got_any) {
            ++bad_frames_;
            return false;
        }
        s.stamp = now_s();
        s.valid = true;
        ++good_frames_;
        std::lock_guard<std::mutex> lk(m_);
        latest_ = s;
        return true;
    }

    std::vector<uint8_t> buf_;
    mutable std::mutex m_;
    RxState latest_;
    uint64_t resync_drops_ = 0, good_frames_ = 0, bad_frames_ = 0;
};

// ------------------------------------------------------------------ 串口
// USB CDC 虚拟串口: 不设波特率(USB 驱动处理), 只做 raw + 非阻塞
inline int open_port(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

inline int open_first_available(const std::vector<std::string>& candidates,
                                std::string* opened = nullptr) {
    for (const auto& c : candidates) {
        int fd = open_port(c);
        if (fd >= 0) {
            if (opened) *opened = c;
            return fd;
        }
    }
    return -1;
}

// ------------------------------------------------------------------ 收发线程
// 发送线程职责(协议第 6 节):
//   1) 固定频率单帧发送, 帧间留空闲间隔(不背靠背连发)
//   2) 启动即开始发送, 哪怕还没有目标
//   3) 看门狗: 目标超过 watchdog_ms 未更新 -> 冻结在最后一个有效角并撤掉开火请求
//      (不能归零: 0 对云台是"猛转到限位"的指令, 比冻结更危险)
//   4) 不发 NaN/Inf
//   5) 只做打包 + write, 不做日志/文件/网络等重活
// 退出时补发若干零帧(协议要求退出/异常/Ctrl-C 都要补发)。
class SerialLink {
public:
    SerialLink() = default;
    ~SerialLink() { stop(); }
    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    // fd < 0 => mock 模式: 不碰真串口, 只跑收发计数, 便于离线验证闭环逻辑
    //
    // 重要: start() **不会**自动发任何东西。必须显式调用 set_target() 之后才会开始
    // 发送。原因是 ID1 是绝对角度, 上位机在收到下位机回传之前根本不知道云台现在
    // 指哪儿; 此时若发一个默认值(尤其是 0,0), 就等于命令云台猛转过去。
    // 实测踩坑: 之前这里初始化成 TxTarget{}(即 yaw=0,pitch=0), 结果每次启动程序
    // 云台都被拽到 0/0, 手动对好的方向全丢。
    bool start(int fd, int send_hz = 100, int watchdog_ms = 500,
               int shutdown_frames = 20) {
        fd_ = fd;
        send_period_ms_ = 1000 / (send_hz > 0 ? send_hz : 1);
        watchdog_ms_ = watchdog_ms;
        shutdown_zero_frames_ = shutdown_frames;
        stop_flag_ = false;
        has_target_ = false;
        tx_frames_ = 0;
        tx_thread_ = std::thread(&SerialLink::tx_loop, this);
        if (fd_ >= 0) rx_thread_ = std::thread(&SerialLink::rx_loop, this);
        return true;
    }

    void stop() {
        if (stop_flag_.exchange(true)) return;
        if (rx_thread_.joinable()) rx_thread_.join();
        if (tx_thread_.joinable()) tx_thread_.join();
        if (fd_ >= 0) {
            // 退出时补发若干帧。补发的必须是**最后一个有效命令角**, 不能归零!
            //
            // 协议文档 6.3 节说"退出/异常/Ctrl-C 都要补发零速度帧", 但那条是写给
            // 导航底盘的速度指令的 —— 速度 0 = 停住, 安全。
            // 自瞄这边 ID1 是**绝对角度**, (0,0) 是一个合法角度, 归零等于命令云台
            // 猛转到 yaw=0 / pitch=0。
            // 实测踩坑: 退出 gimbal_test 后云台从 yaw=-138 被甩到 -24 度,
            // 后续所有测试都指向了错误的方位, 白白排查了好几轮。
            TxTarget hold;
            {
                std::lock_guard<std::mutex> lk(tx_m_);
                hold = target_;
            }
            const float y = zero_on_shutdown_ ? 0.0f : hold.yaw;
            const float p = zero_on_shutdown_ ? 0.0f : hold.pitch;
            for (int i = 0; i < shutdown_zero_frames_; ++i) {
                auto pkt = pack_tx(y, p, 0);
                const ssize_t n = ::write(fd_, pkt.data(), pkt.size());
                (void)n;
                std::this_thread::sleep_for(std::chrono::milliseconds(send_period_ms_));
            }
            ::close(fd_);
            fd_ = -1;
        }
    }

    // 退出时是否归零。默认 false = 保持最后命令角(云台正确行为)。
    // 只有将来驱动底盘速度指令时才该设 true。
    void set_zero_on_shutdown(bool v) { zero_on_shutdown_ = v; }

    // 主线程: 更新要发给下位机的目标(线程安全)
    void set_target(float yaw_deg, float pitch_deg, int32_t fire_flag) {
        std::lock_guard<std::mutex> lk(tx_m_);
        target_.yaw       = yaw_deg;
        target_.pitch     = pitch_deg;
        target_.fire_flag = fire_flag;
        target_.stamp     = now_s();
        has_target_       = true;   // 从这一刻起才开始真正发送
    }

    // 有没有设过目标(没设过就一帧都没发)
    bool has_target() const { return has_target_; }

    // 主线程: 强制回零(例如急停/异常)
    void zero_target() { set_target(0.0f, 0.0f, 0); }

    RxState  state() const { return parser_.snapshot(); }
    bool     mock() const { return fd_ < 0; }
    uint64_t tx_frames() const { return tx_frames_; }
    uint64_t rx_frames() const { return parser_.good_frames(); }
    uint64_t rx_bad_frames() const { return parser_.bad_frames(); }
    uint64_t resync_drops() const { return parser_.resync_drops(); }
    uint64_t watchdog_trips() const { return watchdog_trips_; }

private:
    struct TxTarget {
        float   yaw = 0.0f, pitch = 0.0f;
        int32_t fire_flag = 0;
        double  stamp = 0.0;
    };

    void tx_loop() {
        auto next = std::chrono::steady_clock::now();
        bool warned = false;
        while (!stop_flag_) {
            // 还没设过目标就什么都不发。
            // 关键: 在收到下位机回传之前我们不知道云台当前指哪儿, 此时发任何取值
            // 都可能让云台猛转。宁可晚发, 也不要发错。
            if (!has_target_) {
                if (!warned) {
                    warned = true;
                    std::fprintf(stderr,
                                 "[SERIAL] 尚未设定目标角, 暂不发送(等 set_target)。\n"
                                 "         这是有意的: 不知道云台当前姿态时不发指令更安全。\n");
                }
                next += std::chrono::milliseconds(send_period_ms_);
                std::this_thread::sleep_until(next);
                continue;
            }
            TxTarget t;
            {
                std::lock_guard<std::mutex> lk(tx_m_);
                t = target_;
            }
            if ((now_s() - t.stamp) * 1000.0 > watchdog_ms_) {
                // 看门狗: 控制量太久没更新, 冻结在最后一个有效角上, 并撤掉开火请求。
                // 绝不能归零 —— (0,0) 是一个合法的绝对角度, 等于命令云台转到 0/0。
                if (!watchdog_active_) {
                    watchdog_active_ = true;
                    ++watchdog_trips_;
                }
                t.yaw       = last_tx_.yaw;
                t.pitch     = last_tx_.pitch;
                t.fire_flag = 0;
            } else {
                watchdog_active_ = false;
            }
            // 非有限值同样不能归零, 冻结上一帧
            if (!std::isfinite(t.yaw) || !std::isfinite(t.pitch)) {
                t.yaw       = last_tx_.yaw;
                t.pitch     = last_tx_.pitch;
                t.fire_flag = 0;
            }
            if (fd_ >= 0) {
                auto pkt = pack_tx(t.yaw, t.pitch, t.fire_flag);
                const ssize_t n = ::write(fd_, pkt.data(), pkt.size());
                if (n == static_cast<ssize_t>(pkt.size())) ++tx_frames_;
            } else {
                ++tx_frames_;
            }
            last_tx_ = t;
            next += std::chrono::milliseconds(send_period_ms_);
            std::this_thread::sleep_until(next);
        }
    }

    void rx_loop() {
        std::vector<uint8_t> buf(512);
        while (!stop_flag_) {
            const ssize_t n = ::read(fd_, buf.data(), buf.size());
            if (n > 0) {
                parser_.feed(buf.data(), static_cast<size_t>(n));
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                break;   // 串口断开
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    int fd_ = -1;
    int send_period_ms_ = 10;
    int watchdog_ms_ = 500;
    int shutdown_zero_frames_ = 20;
    bool zero_on_shutdown_ = false;   // 云台: 默认保持最后命令角, 不归零
    std::atomic<bool> has_target_{false};   // 未设目标前不发任何帧
    std::atomic<bool> stop_flag_{true};
    std::atomic<bool> watchdog_active_{false};
    std::atomic<uint64_t> tx_frames_{0};
    std::atomic<uint64_t> watchdog_trips_{0};

    mutable std::mutex tx_m_;
    TxTarget target_;
    TxTarget last_tx_;

    RxParser parser_;
    std::thread rx_thread_, tx_thread_;
};

}  // namespace srm
