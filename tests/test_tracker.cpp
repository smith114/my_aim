// test_tracker.cpp: 三态状态机行为测试
#include "test_util.hpp"
#include "tracker.hpp"

using namespace srm;

static Target mk(float x, float y, float dist = 5.0f, float yaw = 0.0f, float pitch = 0.0f) {
    Target t;
    t.center = cv::Point2f(x, y);
    t.solve.ok = true;
    t.solve.dist = dist;
    t.solve.yaw = yaw;
    t.solve.pitch = pitch;
    return t;
}

// 喂入同一批候选直到锁定, 返回用了几帧; 超过 limit 帧仍未锁定返回 -1。
// 用它而不是写死"3 帧", 这样以后调 TRACK_CONFIRM_FRAMES 不会把测试弄坏
// (之前写死 3, 把确认帧数改成 2 之后测试就红了)。
static int feed_until_locked(Tracker& tr, const std::vector<Target>& c, int limit = 30) {
    for (int i = 0; i < limit; ++i) {
        if (tr.update(c).locked) return i + 1;
    }
    return -1;
}

int main() {
    // ---------------------------------------------------------- 单帧误检不能锁定
    // (前提: 确认帧数 > 1。设为 1 就等于关掉了防误检, 那是有意为之的话本用例不适用)
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        const auto r = tr.update({mk(700, 500)});
        if (TRACK_CONFIRM_FRAMES > 1) {
            CHECK(!r.locked);
            CHECK(tr.state() == TrackState::DETECT);
        }
    }

    // ---------------------------------------------------------- 连续 TRACK_CONFIRM_FRAMES 帧后锁定
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        const int n = feed_until_locked(tr, {mk(700, 500)});
        CHECK(n == TRACK_CONFIRM_FRAMES);
        CHECK(tr.state() == TrackState::TRACK);
        CHECK(tr.has_target());
        CHECK_NEAR(tr.target().solve.dist, 5.0f, 1e-6);
        std::printf("  [确认] TRACK_CONFIRM_FRAMES=%d, 实测 %d 帧锁定\n",
                    TRACK_CONFIRM_FRAMES, n);
    }

    // ---------------------------------------------------------- 这是修复过的 bug:
    // 静止的同一个误检, 中间断档后不能靠"历史累加"蒙混过关
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        // 先喂到差一帧就锁定
        for (int i = 0; i + 1 < TRACK_CONFIRM_FRAMES; ++i) {
            CHECK(!tr.update({mk(700, 500)}).locked);
        }
        // 断 1 帧
        CHECK(!tr.update({}).locked);
        // 断档之后的累加必须作废 -> 还得再来满 TRACK_CONFIRM_FRAMES 帧
        const int n = feed_until_locked(tr, {mk(700, 500)});
        CHECK(n == TRACK_CONFIRM_FRAMES);
        std::printf("  [断档] 断 1 帧后重新需要 %d 帧锁定(等于确认帧数, 说明累加已作废)\n",
                    n);
    }

    // ---------------------------------------------------------- 跳太远的候选不算同一目标
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        CHECK(feed_until_locked(tr, {mk(700, 500)}) > 0);
        CHECK(tr.state() == TrackState::TRACK);
        // 目标突然跳到 300px 外: 超过关联门限, 应判为丢失而不是跟过去
        const auto r = tr.update({mk(1000, 500)});
        CHECK(!r.locked);
        CHECK(tr.state() == TrackState::LOST);
    }

    // ---------------------------------------------------------- 短暂丢失后重新关联
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        tr.update({mk(700, 500)});
        tr.update({mk(700, 500)});
        CHECK(tr.update({mk(700, 500)}).locked);
        // 丢 2 帧
        CHECK(!tr.update({}).locked);
        CHECK(!tr.update({}).locked);
        CHECK(tr.state() == TrackState::LOST);
        // 目标回来(小幅移动) -> 立即恢复锁定, 不需要重新确认
        const auto r = tr.update({mk(705, 498)});
        CHECK(r.locked);
        CHECK(tr.state() == TrackState::TRACK);
    }

    // ---------------------------------------------------------- 丢太久回到 DETECT
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        CHECK(feed_until_locked(tr, {mk(700, 500)}) > 0);
        for (int i = 0; i < MAX_LOST_FRAMES; ++i) tr.update({});
        CHECK(tr.state() == TrackState::DETECT);
        // 回到 DETECT 后必须重新连续确认
        CHECK(!tr.update({mk(700, 500)}).locked);
    }

    // ---------------------------------------------------------- 多目标时的选择
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        // 两个候选: 靠近画面中心的应该被优先选中
        const std::vector<Target> two{mk(100, 100), mk(720, 540)};
        const int n = feed_until_locked(tr, two);
        CHECK(n == TRACK_CONFIRM_FRAMES);
        CHECK(tr.state() == TrackState::TRACK);
        CHECK(tr.has_target());
        CHECK_NEAR(tr.target().center.x, 720.0f, 1e-3);   // 选的是靠中心那个

        // 锁定后目标移动, 应从候选里挑离锁定中心最近的那个
        const auto r2 = tr.update({mk(200, 200), mk(730, 545)});
        CHECK(r2.locked);
        CHECK(r2.index == 1);
    }

    // ---------------------------------------------------------- reset 后回到初始态
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        CHECK(feed_until_locked(tr, {mk(700, 500)}) > 0);
        tr.reset();
        CHECK(tr.state() == TrackState::DETECT);
        CHECK(tr.lost_count() == 0);
        CHECK(tr.confirm_count() == 0);
        CHECK(!tr.update({mk(700, 500)}).locked);
    }

    // ---------------------------------------------------------- 状态名
    {
        Tracker tr;
        CHECK(std::string(tr.state_name()) == "DETECT");
    }

    TEST_MAIN_END();
}
