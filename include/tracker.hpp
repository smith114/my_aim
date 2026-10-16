// tracker.hpp: 目标跟踪 —— DETECT / TRACK / LOST 三态状态机
//
// 为什么要状态机:
//   单帧检测必然有误检和漏检。直接"有检测就驱动云台"会让云台被误检带着乱抖,
//   偶尔漏一帧又会让框闪掉。所以:
//     DETECT : 连续 N 帧盯住同一位置才确认锁定 -> 防单帧误检驱动云台
//     TRACK  : 只跟已锁定目标附近的候选(关联门限), 跳太远视为误检/丢失
//     LOST   : 短暂丢失保持锁定信息, 云台不动; 超过 MAX_LOST_FRAMES 才回 DETECT
//
// 与前版的区别:
//   1) 确认计数在出现检测断档后重新起算(前版只增不减, 静止误检几十帧后照样确认)
//   2) 首次搜索时选离画面中心最近的候选, 而不是任取一个
//   3) 候选与状态机解耦, 可直接单元测试
#pragma once

#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "solver.hpp"

namespace srm {

enum class TrackState { DETECT, TRACK, LOST };

// 一帧里的一个候选目标
struct Target {
    cv::Point2f  center;              // 像素中心
    SolveResult  solve;               // 解算结果
    int          color = 0;
};

// 状态机输出
struct TrackResult {
    bool   locked = false;   // 是否锁定(可驱动云台)
    int    index  = -1;      // 命中 candidates 的下标
    Target target;           // 命中目标(index >= 0 时有效)
};

class Tracker {
public:
    // 每帧调用一次; candidates 为已通过解算有效性检查的候选
    TrackResult update(const std::vector<Target>& candidates) {
        TrackResult out;

        // ---------------- 无检测 ----------------
        if (candidates.empty()) {
            ++lost_count_;
            ++gap_frames_;
            had_gap_ = true;                 // 出现过断档, DETECT 的确认累加要作废
            if (lost_count_ >= MAX_LOST_FRAMES) {
                state_ = TrackState::DETECT;
                confirm_count_ = 0;
                have_last_ = false;
            } else if (state_ == TrackState::TRACK) {
                state_ = TrackState::LOST;   // 短暂丢失: 保持锁定信息
            }
            out.locked = false;
            return out;
        }

        // ---------------- 已锁定: 就近关联 ----------------
        if (state_ == TrackState::TRACK || state_ == TrackState::LOST) {
            int   best   = -1;
            float best_d = 1e9f;
            for (size_t i = 0; i < candidates.size(); ++i) {
                const float d = cv::norm(candidates[i].center - center_);
                if (d < best_d) {
                    best_d = d;
                    best   = static_cast<int>(i);
                }
            }
            if (best >= 0 && best_d <= TRACK_ASSOC_PX) {
                center_     = candidates[best].center;
                last_seen_  = candidates[best];
                state_      = TrackState::TRACK;
                lost_count_ = 0;
                gap_frames_ = 0;
                had_gap_    = false;
                out.locked  = true;
                out.index   = best;
                out.target  = last_seen_;
                return out;
            }
            // 联不上: 这一帧算丢失
            ++lost_count_;
            had_gap_ = true;
            if (lost_count_ >= MAX_LOST_FRAMES) {
                state_ = TrackState::DETECT;
                confirm_count_ = 0;
                have_last_ = false;
            } else {
                state_ = TrackState::LOST;
            }
            out.locked = false;
            return out;
        }

        // ---------------- DETECT: 需要连续确认 ----------------
        int best = choose_for_detect(candidates);
        const float assoc = have_last_
                                ? cv::norm(candidates[best].center - last_cand_)
                                : 0.0f;
        if (have_last_ && assoc <= TRACK_ASSOC_PX && !had_gap_) {
            ++confirm_count_;
        } else {
            // 与前次候选对不上, 或中间有断档: 重新起算
            confirm_count_ = 1;
        }
        last_cand_  = candidates[best].center;
        have_last_  = true;
        had_gap_    = false;
        gap_frames_ = 0;

        if (confirm_count_ >= TRACK_CONFIRM_FRAMES) {
            state_      = TrackState::TRACK;
            lost_count_ = 0;
            center_     = candidates[best].center;
            last_seen_  = candidates[best];
            out.locked  = true;
            out.index   = best;
            out.target  = last_seen_;
            return out;
        }
        out.locked = false;
        return out;
    }

    // 告诉跟踪器图像尺寸, 用于 DETECT 首帧"选离画面中心最近的目标"
    void set_frame_size(const cv::Size& s) { frame_size_ = s; }

    TrackState state() const { return state_; }
    int  lost_count() const { return lost_count_; }
    int  confirm_count() const { return confirm_count_; }
    bool has_target() const { return state_ == TrackState::TRACK; }
    const Target& target() const { return last_seen_; }

    void reset() {
        state_         = TrackState::DETECT;
        lost_count_    = 0;
        confirm_count_ = 0;
        gap_frames_    = 0;
        have_last_     = false;
        had_gap_       = false;
    }

    const char* state_name() const {
        switch (state_) {
            case TrackState::TRACK: return "TRACK";
            case TrackState::LOST:  return "LOST";
            default:                return "DETECT";
        }
    }

    // 供测试观察
    cv::Point2f locked_center() const { return center_; }

private:
    // DETECT 阶段选谁: 有历史就选帧间位移最小的, 否则选离画面中心最近的
    int choose_for_detect(const std::vector<Target>& candidates) const {
        int   best   = 0;
        float best_d = 1e9f;
        if (have_last_) {
            for (size_t i = 0; i < candidates.size(); ++i) {
                const float d = cv::norm(candidates[i].center - last_cand_);
                if (d < best_d) {
                    best_d = d;
                    best   = static_cast<int>(i);
                }
            }
            return best;
        }
        const cv::Point2f img_center(static_cast<float>(frame_size_.width) * 0.5f,
                                     static_cast<float>(frame_size_.height) * 0.5f);
        for (size_t i = 0; i < candidates.size(); ++i) {
            const float d = cv::norm(candidates[i].center - img_center);
            if (d < best_d) {
                best_d = d;
                best   = static_cast<int>(i);
            }
        }
        return best;
    }

    TrackState  state_ = TrackState::DETECT;
    cv::Point2f center_{0.0f, 0.0f};      // 锁定目标中心
    cv::Point2f last_cand_{0.0f, 0.0f};   // DETECT 阶段上一帧候选
    bool        have_last_    = false;
    cv::Size    frame_size_{1440, 1080};
    Target      last_seen_;
    int         lost_count_    = 0;
    int         confirm_count_ = 0;
    int         gap_frames_    = 0;
    bool        had_gap_       = false;
};

}  // namespace srm
