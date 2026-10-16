// hik_camera.hpp: 海康MVS SDK取流 (严格像素格式处理, 防越界崩溃)
#pragma once
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>
#include <iostream>
#include <cstring>

static const char* fmt_name(int f) {
    switch (f) {
        case PixelType_Gvsp_Mono8:         return "Mono8";
        case PixelType_Gvsp_BGR8_Packed:   return "BGR8";
        case PixelType_Gvsp_RGB8_Packed:   return "RGB8";
        case PixelType_Gvsp_BayerRG8:      return "BayerRG8";
        case PixelType_Gvsp_BayerBG8:      return "BayerBG8";
        case PixelType_Gvsp_BayerGR8:      return "BayerGR8";
        case PixelType_Gvsp_BayerGB8:      return "BayerGB8";
        default:                           return "other";
    }
}

class HikCamera {
public:
    HikCamera() : handle_(nullptr), grabbing_(false), fmt_(-1), bayer_code_(-1), bayer_detected_(false) {}

    // 循环切换 Bayer 转 BGR 模式, 实机颜色反时按T试到正确
    void cycle_bayer() {
        static const int codes[] = {
            cv::COLOR_BayerRG2BGR, cv::COLOR_BayerBG2BGR,
            cv::COLOR_BayerGR2BGR, cv::COLOR_BayerGB2BGR
        };
        static const char* names[] = {"RG", "BG", "GR", "GB"};
        int idx = 0;
        for (int i = 0; i < 4; i++) if (codes[i] == bayer_code_) idx = i;
        idx = (idx + 1) % 4;
        bayer_code_ = codes[idx];
        std::cout << "[BAYER] switch to " << names[idx] << "\n";
    }

    // 第一帧自动探测正确Bayer模式: 4种转换统计蓝/红像素数, 最多者颜色最饱和(正确转换)
    static int detect_bayer_code(const cv::Mat& bay) {
        static const int codes[4] = { cv::COLOR_BayerRG2BGR, cv::COLOR_BayerBG2BGR,
                                      cv::COLOR_BayerGR2BGR, cv::COLOR_BayerGB2BGR };
        static const char* names[4] = {"RG", "BG", "GR", "GB"};
        int best = codes[0], best_cnt = -1, best_i = 0;
        for (int i = 0; i < 4; i++) {
            cv::Mat bgr, hsv;
            cv::cvtColor(bay, bgr, codes[i]);
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            cv::Mat bm, r1, r2, rm;
            cv::inRange(hsv, cv::Scalar(100, 80, 60), cv::Scalar(130, 255, 255), bm);
            cv::inRange(hsv, cv::Scalar(0, 80, 60), cv::Scalar(10, 255, 255), r1);
            cv::inRange(hsv, cv::Scalar(170, 80, 60), cv::Scalar(180, 255, 255), r2);
            cv::bitwise_or(r1, r2, rm);
            int cnt = cv::countNonZero(bm) + cv::countNonZero(rm);
            if (cnt > best_cnt) { best_cnt = cnt; best = codes[i]; best_i = i; }
        }
        std::cout << "[BAYER] auto-detect -> " << names[best_i] << " (pix=" << best_cnt << ")\n";
        return best;
    }

    bool open() {
        MV_CC_DEVICE_INFO_LIST devices;
        memset(&devices, 0, sizeof(devices));
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &devices);
        if (ret != MV_OK || devices.nDeviceNum == 0) {
            std::cerr << "[HIK] no camera found\n";
            return false;
        }
        std::cout << "[HIK] found " << devices.nDeviceNum << " camera(s)\n";

        ret = MV_CC_CreateHandle(&handle_, devices.pDeviceInfo[0]);
        if (ret != MV_OK) { std::cerr << "[HIK] create handle fail: " << ret << "\n"; return false; }

        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) { std::cerr << "[HIK] open device fail: " << ret << "\n"; return false; }

        // 显式配置: 关闭触发 + 连续采集 (StartGrabbing 前置条件, 失败仅告警不中断)
        int r1 = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
        int r2 = MV_CC_SetEnumValue(handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        if (r1 != MV_OK || r2 != MV_OK)
            std::cerr << "[HIK] warn: trigger/acq set ret=" << r1 << "/" << r2 << " (keep camera default)\n";

        // 关键: 相机可能被保存在 BGR8(带宽大户, CS016 取流会资源申请失败), 主动切回带宽友好格式
        // 优先级: BayerRG8 -> BayerGB8 -> Mono8
        // 必须 Bayer 优先: Mono8 灰度帧上灯条颜色分离(R-B/B-R)恒为0, verify_lightbars 会滤掉全部目标
        // (实测实机 verify_pass=0 即此因); Bayer 8bit 带宽与 Mono8 相同, read() 侧转 BGR 彩色
        const int try_list[] = { PixelType_Gvsp_BayerBG8, PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerRG8, PixelType_Gvsp_Mono8 };
        fmt_ = -1;
        for (int f : try_list) {
            int r = MV_CC_SetEnumValue(handle_, "PixelFormat", f);
            std::cout << "[HIK] try set " << fmt_name(f) << " -> ret " << r << "\n";
            if (r == MV_OK) { fmt_ = f; break; }
        }
        if (fmt_ < 0) {
            std::cerr << "[HIK] warn: all format set failed, keep camera default\n";
            MVCC_ENUMVALUE en;
            memset(&en, 0, sizeof(en));
            if (MV_CC_GetEnumValue(handle_, "PixelFormat", &en) == MV_OK) {
                fmt_ = static_cast<int>(en.nCurValue);
                std::cout << "[HIK] current pixel format = " << fmt_name(fmt_) << " (" << fmt_ << ")\n";
            }
        } else {
            std::cout << "[HIK] using pixel format = " << fmt_name(fmt_) << "\n";
        }

        // 画面偏暗: 开自动曝光+自动增益 (失败不阻断, 相机若不支持自动忽略)
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 2);  // 2=Continuous
        MV_CC_SetEnumValue(handle_, "GainAuto", 2);

        // 实机实测(2026-10-15): 固定 BayerBG2BGR 颜色正确
        // 海康SDK Bayer命名与OpenCV标准相反: SDK的RG8数据实际是BGGR布局
        // T键可现场切换兜底 (cycle_bayer)
        bayer_code_ = cv::COLOR_BayerBG2BGR;

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            std::cerr << "[HIK] start grabbing fail: " << ret << " (0x" << std::hex << ret << std::dec
                      << ") = 资源申请失败; 检查: USB3口/线, 是否有其他进程占用相机\n";
            return false;
        }
        grabbing_ = true;
        return true;
    }

    bool read(cv::Mat& frame) {
        if (!grabbing_) return false;
        MV_FRAME_OUT fi;
        memset(&fi, 0, sizeof(fi));
        int ret = MV_CC_GetImageBuffer(handle_, &fi, 1000);
        if (ret != MV_OK) return false;

        int w = fi.stFrameInfo.nWidth;
        int h = fi.stFrameInfo.nHeight;
        int fmt = fi.stFrameInfo.enPixelType;   // 以实际帧格式为准
        if (w <= 0 || h <= 0) { MV_CC_FreeImageBuffer(handle_, &fi); return false; }

        switch (fmt) {
        case PixelType_Gvsp_BGR8_Packed:
            frame = cv::Mat(h, w, CV_8UC3, fi.pBufAddr).clone();
            break;
        case PixelType_Gvsp_RGB8_Packed:
            frame = cv::Mat(h, w, CV_8UC3, fi.pBufAddr).clone();
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            break;
        case PixelType_Gvsp_Mono8:
            frame = cv::Mat(h, w, CV_8UC1, fi.pBufAddr).clone();
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            break;
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_BayerBG8:
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_BayerGB8: {
            cv::Mat bay(h, w, CV_8UC1, fi.pBufAddr);
            if (!bayer_detected_) {   // 第一帧自动探测正确Bayer模式(不再猜)
                bayer_code_ = detect_bayer_code(bay);
                bayer_detected_ = true;
            }
            cv::cvtColor(bay, frame, bayer_code_);
            break;
        }
        default:
            std::cerr << "[HIK] unsupported fmt=" << fmt << ", skip frame\n";
            MV_CC_FreeImageBuffer(handle_, &fi);
            return false;
        }

        MV_CC_FreeImageBuffer(handle_, &fi);
        return true;
    }

    // 显式关闭, 便于 main 在退出前释放相机(而不是等析构)
    void close() {
        if (grabbing_) {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }
        if (handle_) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
    }

    ~HikCamera() { close(); }

    // 当前 Bayer 转换模式名, 便于日志确认
    const char* bayer_name() const {
        switch (bayer_code_) {
            case cv::COLOR_BayerRG2BGR: return "RG";
            case cv::COLOR_BayerBG2BGR: return "BG";
            case cv::COLOR_BayerGR2BGR: return "GR";
            case cv::COLOR_BayerGB2BGR: return "GB";
            default:                    return "?";
        }
    }

private:
    void* handle_;
    bool grabbing_;
    int fmt_;
    int bayer_code_;       // cvtColor Bayer code, T键现场切换
    bool bayer_detected_;  // 第一帧自动探测已完成
};