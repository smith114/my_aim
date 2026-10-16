// test_protocol.cpp: 通信协议一致性测试
//
// 依据《SRM 校内赛自瞄通信协议》逐字段核对:
//   上->下 18B: body_len=16, ID1, yaw, pitch, ID2, fire_flag
//   下->上 30B: body_len=28, ID1, yaw,pitch,roll,mode,color, ID2, bullet_speed
// 并验证流式解帧在"半帧/多帧/粘连/失步"情况下的行为。
#include "protocol.hpp"
#include "test_util.hpp"

#include <vector>

using namespace srm;

// 按协议手搓一帧下位机 -> 上位机 的 30 字节帧
static std::vector<uint8_t> build_rx_frame(float yaw, float pitch, float roll,
                                           int32_t mode, int32_t color,
                                           float bullet_speed) {
    std::vector<uint8_t> b;
    put_i16(b, 28);          // body_len
    put_i16(b, 1);           // ID 1
    put_f32(b, yaw);
    put_f32(b, pitch);
    put_f32(b, roll);
    put_i32(b, mode);
    put_i32(b, color);
    put_i16(b, 2);           // ID 2
    put_f32(b, bullet_speed);
    return b;
}

int main() {
    // ---------------------------------------------------------- 上行帧
    {
        const auto pkt = pack_tx(12.5f, -3.25f, 0);
        CHECK(pkt.size() == kFrameLenTx);
        CHECK(pkt.size() == 18);
        CHECK(get_i16(pkt.data() + 0) == 16);
        CHECK(get_i16(pkt.data() + 2) == 1);
        CHECK_NEAR(get_f32(pkt.data() + 4), 12.5f, 1e-6);
        CHECK_NEAR(get_f32(pkt.data() + 8), -3.25f, 1e-6);
        CHECK(get_i16(pkt.data() + 12) == 2);
        CHECK(get_i32(pkt.data() + 14) == 0);
    }
    {
        const auto pkt = pack_tx(1.0f, 2.0f, 12345);
        CHECK(get_i32(pkt.data() + 14) == 12345);
        CHECK(pkt[14] == 0x39);   // 小端: 12345 = 0x3039 -> 低字节 0x39
        CHECK(pkt[15] == 0x30);
    }

    // ---------------------------------------------------------- 下行帧
    {
        RxParser p;
        const auto f = build_rx_frame(88.5f, 12.25f, 0.5f, 0, 7, 22.75f);
        CHECK(f.size() == kFrameLenRx);
        CHECK(f.size() == 30);
        CHECK(p.feed(f.data(), f.size()) == 1);
        const RxState s = p.snapshot();
        CHECK(s.valid);
        CHECK_NEAR(s.yaw, 88.5f, 1e-6);
        CHECK_NEAR(s.pitch, 12.25f, 1e-6);
        CHECK_NEAR(s.roll, 0.5f, 1e-6);
        CHECK(s.mode == 0);
        CHECK(s.robot_id == 7);
        CHECK(s.has_bullet_speed);
        CHECK_NEAR(s.bullet_speed, 22.75f, 1e-6);
        CHECK(p.good_frames() == 1);
        CHECK(p.bad_frames() == 0);
        CHECK(p.resync_drops() == 0);
        CHECK(p.pending() == 0);
    }

    // ---------------------------------------------------------- 半帧流式
    {
        RxParser p;
        const auto f = build_rx_frame(10.0f, 20.0f, 0.0f, 0, 3, 22.0f);
        // 逐字节喂, 前 29 字节都不应该产生完整帧
        for (size_t i = 0; i + 1 < f.size(); ++i) {
            CHECK(p.feed(f.data() + i, 1) == 0);
        }
        CHECK(p.feed(f.data() + f.size() - 1, 1) == 1);
        const RxState s = p.snapshot();
        CHECK_NEAR(s.yaw, 10.0f, 1e-6);
        CHECK_NEAR(s.pitch, 20.0f, 1e-6);
        CHECK(s.robot_id == 3);
    }

    // ---------------------------------------------------------- 粘连多帧
    {
        RxParser p;
        std::vector<uint8_t> stream;
        for (int i = 0; i < 5; ++i) {
            const auto f = build_rx_frame(static_cast<float>(i), 0.0f, 0.0f, 0, i, 22.0f);
            stream.insert(stream.end(), f.begin(), f.end());
        }
        CHECK(p.feed(stream.data(), stream.size()) == 5);
        CHECK(p.snapshot().robot_id == 4);
        CHECK(p.good_frames() == 5);
        CHECK(p.pending() == 0);
    }

    // ---------------------------------------------------------- 半帧粘连
    {
        RxParser p;
        const auto f1 = build_rx_frame(1.0f, 0.0f, 0.0f, 0, 1, 22.0f);
        const auto f2 = build_rx_frame(2.0f, 0.0f, 0.0f, 0, 2, 22.0f);
        std::vector<uint8_t> part(f1.begin(), f1.begin() + 17);
        CHECK(p.feed(part.data(), part.size()) == 0);
        CHECK(p.pending() == 17);
        std::vector<uint8_t> rest(f1.begin() + 17, f1.end());
        rest.insert(rest.end(), f2.begin(), f2.begin() + 9);
        CHECK(p.feed(rest.data(), rest.size()) == 1);   // f1 完成
        CHECK(p.snapshot().robot_id == 1);
        CHECK(p.pending() == 9);
        const std::vector<uint8_t> rest2(f2.begin() + 9, f2.end());
        CHECK(p.feed(rest2.data(), rest2.size()) == 1);  // f2 完成
        CHECK(p.snapshot().robot_id == 2);
    }

    // ---------------------------------------------------------- 非法长度 -> 丢缓存
    {
        RxParser p;
        const uint8_t junk[] = {0xA5, 0x00, 0x11, 0x22, 0x33};
        p.feed(junk, sizeof(junk));
        CHECK(p.resync_drops() == 1);
        CHECK(p.pending() == 0);
        // 丢完缓存后应能重新正常接收
        const auto f = build_rx_frame(5.0f, 6.0f, 0.0f, 0, 9, 22.0f);
        CHECK(p.feed(f.data(), f.size()) == 1);
        CHECK_NEAR(p.snapshot().yaw, 5.0f, 1e-6);
        CHECK(p.snapshot().robot_id == 9);
    }

    // ---------------------------------------------------------- 未知 ID -> 坏帧, 不崩
    {
        RxParser p;
        std::vector<uint8_t> b;
        put_i16(b, 4);
        put_i16(b, 99);      // 未知 ID
        put_i16(b, 0);
        put_i16(b, 0);       // 凑满 body_len
        // 注意: body_len=4 != 28, 会被当失步丢弃; 这里验证不崩溃
        p.feed(b.data(), b.size());
        CHECK(p.resync_drops() + p.bad_frames() >= 1);
    }

    // ---------------------------------------------------------- 默认弹速
    {
        const RxState s;
        CHECK_NEAR(s.bullet_speed, kDefaultBulletSpeed, 1e-6);
        CHECK(!s.has_bullet_speed);
        CHECK(!s.valid);
    }

    // ---------------------------------------------------------- 串口设备解析失败要返回 -1
    {
        const int fd = open_port("/dev/definitely-not-exist-9x8y7z");
        CHECK(fd < 0);
        CHECK(open_first_available({"/dev/nope-1", "/dev/nope-2"}) < 0);
    }

    TEST_MAIN_END();
}
