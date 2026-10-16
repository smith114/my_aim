// hik_camera.hpp: 海康MVS SDK取流 (严格像素格式处理, 防越界崩溃)
#pragma once
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>
#include <cstring>
#include <iostream>
#include <string>

#include "config.hpp"

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
    // Bayer 相位名, 用于日志
    static const char* bayer_code_name(int code) {
        switch (code) {
            case cv::COLOR_BayerRG2BGR: return "rg";
            case cv::COLOR_BayerBG2BGR: return "bg";
            case cv::COLOR_BayerGR2BGR: return "gr";
            case cv::COLOR_BayerGB2BGR: return "gb";
            default:                    return "?";
        }
    }

    // 名字 -> OpenCV Bayer 转换码, 无法识别时回退到 config 里的默认值
    static int bayer_from_name(const std::string& n) {
        if (n == "rg") return cv::COLOR_BayerRG2BGR;
        if (n == "bg") return cv::COLOR_BayerBG2BGR;
        if (n == "gr") return cv::COLOR_BayerGR2BGR;
        if (n == "gb") return cv::COLOR_BayerGB2BGR;
        std::cerr << "[BAYER] 无法识别的相位名 '" << n << "', 用默认 "
                  << BAYER_DEFAULT_NAME << "\n";
        if (std::string(BAYER_DEFAULT_NAME) == "rg") return cv::COLOR_BayerRG2BGR;
        if (std::string(BAYER_DEFAULT_NAME) == "gr") return cv::COLOR_BayerGR2BGR;
        if (std::string(BAYER_DEFAULT_NAME) == "gb") return cv::COLOR_BayerGB2BGR;
        return cv::COLOR_BayerBG2BGR;
    }

    HikCamera() : handle_(nullptr), grabbing_(false), fmt_(-1),
                  bayer_code_(bayer_from_name(BAYER_DEFAULT_NAME)) {}

    // 启动时指定 Bayer 相位(--bayer 参数), 覆盖 config.hpp 的默认值
    void set_bayer(int code) {
        bayer_code_ = code;
        std::cout << "[BAYER] 使用 " << bayer_code_name(code) << "\n";
    }

    // 循环切换 Bayer 转 BGR 模式, 实机颜色不对时按 T 逐个试
    void cycle_bayer() {
        static const int codes[] = {
            cv::COLOR_BayerRG2BGR, cv::COLOR_BayerBG2BGR,
            cv::COLOR_BayerGR2BGR, cv::COLOR_BayerGB2BGR
        };
        int idx = 0;
        for (int i = 0; i < 4; i++) if (codes[i] == bayer_code_) idx = i;
        idx = (idx + 1) % 4;
        bayer_code_ = codes[idx];
        std::cout << "[BAYER] switch to " << bayer_code_name(bayer_code_) << "\n";
    }

    // Bayer 相位的自动判定 —— 结论: 在实机上**不可靠**, 不要依赖它。
    //
    // 试过三种判据, 在同一台相机、同一场景下都会给出不同答案:
    //   1) "哪种转换解出的蓝/红像素最多" -> 两次运行分别给出 RG 和 BG
    //   2) "去马赛克后 corr(R,G) 最小"   -> 灰阶场景下四种排列都是 -1, 无法区分
    //   3) "同色相邻列方差最小"          -> colΔ 偶/奇 = 4.96/5.33, 差异只有噪声量级
    //
    // 所以改成: 取 config.hpp 里的 BAYER_DEFAULT_NAME 作为默认值, 现场用
    // --bayer 覆盖, 或者运行中按 T 键逐个试, 以**实车看到的颜色**为准。
    // 判对错最直接的办法: 对着蓝靶标跑 `detect_probe -v`,
    // 看灯条 BGR 均值是否满足 B 明显大于 G 和 R。

    bool open() {
        MV_CC_DEVICE_INFO_LIST devices;
        memset(&devices, 0, sizeof(devices));
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &devices);
        if (ret != MV_OK) {
            std::cerr << "[HIK] MV_CC_EnumDevices 失败 ret=0x" << std::hex << ret << std::dec
                      << "\n";
            return false;
        }
        if (devices.nDeviceNum == 0) {
            // 这个错误最常见的真实原因不是"没插相机", 而是链到了不带 USB3 传输层的
            // 精简版 libMvCameraControl.so (只有几 MB 的那种)。完整版在 /opt/MVS。
            std::cerr << "[HIK] 枚举到 0 个相机。排查顺序:\n"
                         "  1) lsusb | grep -i hikrobot   看看 USB 层有没有认到相机\n"
                         "  2) 上面这条有、这里却是 0 -> 多半是链接了精简版 SDK;\n"
                         "     完整 MVS 在 /opt/MVS/lib/64, 用 ldd 确认实际加载的是哪个:\n"
                         "       ldd ./build/my_aim | grep -i mv\n"
                         "  3) 确认 /dev/bus/usb/*/* 对本用户可读写, 或把用户加进 video 组\n";
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

        // 曝光/增益: 默认用**固定值**, 不用连续自动。
        // 实测(10-16)连续自动曝光在"暗背景 + 极亮 LED"的场景里会来回振荡,
        // 表现为同一目标逐帧忽有忽无, 跟踪器因此咬不住锁。
        if (srm::CAM_USE_AUTO_EXPOSURE) {
            MV_CC_SetEnumValue(handle_, "ExposureAuto", 2);   // 2 = Continuous
            MV_CC_SetEnumValue(handle_, "GainAuto", 2);
            std::cout << "[HIK] 曝光: 连续自动 (不推荐, 易振荡)\n";
        } else {
            // 必须先把自动关掉, 否则手动值会被自动算法立刻覆盖
            MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);   // 0 = Off
            MV_CC_SetEnumValue(handle_, "GainAuto", 0);
            const int r1 = MV_CC_SetFloatValue(handle_, "ExposureTime",
                                               srm::CAM_EXPOSURE_US);
            const int r2 = MV_CC_SetFloatValue(handle_, "Gain", srm::CAM_GAIN_DB);
            std::cout << "[HIK] 曝光: 固定 " << srm::CAM_EXPOSURE_US << " us, 增益 "
                      << srm::CAM_GAIN_DB << " dB (ret=" << r1 << "/" << r2 << ")\n";
            if (r1 != MV_OK || r2 != MV_OK) {
                std::cerr << "[HIK] warn: 曝光/增益设置失败(ret=" << r1 << "/" << r2
                          << "), 相机可能不支持手动模式\n";
            }
        }

        std::cout << "[BAYER] 初始相位 " << bayer_code_name(bayer_code_)
                  << " (颜色不对按 T 键切换, 或用 --bayer rg|bg|gr|gb 指定)\n";

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
            const cv::Mat bay(h, w, CV_8UC1, fi.pBufAddr);
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
    const char* bayer_name() const { return bayer_code_name(bayer_code_); }

private:
    void* handle_;
    bool grabbing_;
    int fmt_;
    int bayer_code_;       // cvtColor Bayer code, T键现场切换
};