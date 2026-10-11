// tracker.hpp: DETECT/TRACK/LOST 三态状态机
#pragma once
#include <opencv2/opencv.hpp>
#include "config.hpp"

enum class TrackState { DETECT, TRACK, LOST };

struct Tracker {
    TrackState state = TrackState::DETECT;
    cv::Point2f center;
    float dist = 0, yaw = 0, pitch = 0;
    int lost_count = 0;

    // 每帧喂入检测到的候选(可能多个), 选最优更新
    void update(const std::vector<cv::Point2f>& cands,
                const std::vector<PnPResult>& results) {
        if (cands.empty()) {
            lost_count++;
            if (lost_count >= MAX_LOST_FRAMES) state = TrackState::DETECT;
            return;  // 保持上一帧值
        }
        // 选最优: TRACK 时选离 center 近的, DETECT 时选最近的
        int best = 0;
        if (state == TrackState::TRACK) {
            float bd = 1e9;
            for (size_t i = 0; i < cands.size(); i++) {
                float dd = cv::norm(cands[i] - center);
                if (dd < bd) { bd = dd; best = i; }
            }
        } else {
            float bz = 1e9;
            for (size_t i = 0; i < results.size(); i++) {
                if (results[i].dist < bz) { bz = results[i].dist; best = i; }
            }
        }
        center = cands[best];
        dist = results[best].dist;
        yaw = results[best].yaw;
        pitch = results[best].pitch;
        state = TrackState::TRACK;
        lost_count = 0;
    }
};