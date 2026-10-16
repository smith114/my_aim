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

int main() {
    // ---------------------------------------------------------- 单帧误检不能锁定
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        auto r = tr.update({mk(700, 500)});
        CHECK(!r.locked);
        CHECK(tr.state() == TrackState::DETECT);
    }

    // ---------------------------------------------------------- 连续 3 帧锁定
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        CHECK(!tr.update({mk(700, 500)}).locked);
        CHECK(!tr.update({mk(702, 501)}).locked);
        const auto r = tr.update({mk(704, 502)});
        CHECK(r.locked);
        CHECK(r.index == 0);
        CHECK(tr.state() == TrackState::TRACK);
        CHECK_NEAR(r.target.solve.dist, 5.0f, 1e-6);
    }

    // ---------------------------------------------------------- 这是修复过的 bug:
    // 静止的同一个误检, 中间断档后不能靠"历史累加"蒙混过关
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        CHECK(!tr.update({mk(700, 500)}).locked);
        CHECK(!tr.update({mk(700, 500)}).locked);
        // 断 1 帧
        CHECK(!tr.update({}).locked);
        // 再来一帧, 确认计数必须重新起算 -> 还不能锁定
        CHECK(!tr.update({mk(700, 500)}).locked);
        CHECK(!tr.update({mk(700, 500)}).locked);
        CHECK(tr.update({mk(700, 500)}).locked);   // 重新连续 3 帧才锁定
    }

    // ---------------------------------------------------------- 跳太远的候选不算同一目标
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        tr.update({mk(700, 500)});
        tr.update({mk(700, 500)});
        CHECK(tr.update({mk(700, 500)}).locked);
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
        tr.update({mk(700, 500)});
        tr.update({mk(700, 500)});
        CHECK(tr.update({mk(700, 500)}).locked);
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
        tr.update({mk(100, 100), mk(720, 540)});
        tr.update({mk(100, 100), mk(720, 540)});
        const auto r = tr.update({mk(100, 100), mk(720, 540)});
        CHECK(r.locked);
        CHECK(r.index == 1);
        CHECK_NEAR(r.target.center.x, 720.0f, 1e-3);

        // 锁定后目标移动, 应从候选里挑离锁定中心最近的那个
        const auto r2 = tr.update({mk(200, 200), mk(730, 545)});
        CHECK(r2.locked);
        CHECK(r2.index == 1);
    }

    // ---------------------------------------------------------- reset 后回到初始态
    {
        Tracker tr;
        tr.set_frame_size(cv::Size(1440, 1080));
        tr.update({mk(700, 500)});
        tr.update({mk(700, 500)});
        CHECK(tr.update({mk(700, 500)}).locked);
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
