// S6 v2: 多装甲板选优 + 状态机跟踪
// 策略: 跟踪中优先保持连续性(离track_center最近), 重新搜索时选距离最近
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length, width, angle;
};

struct Armor {
    cv::Point2f center;
    float dist, yaw, pitch;
};

enum State { DETECT, TRACK, LOST };
const char* STATE_NAME[] = {"DETECT", "TRACK", "LOST"};

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " <video>\n"; return -1; }
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) { std::cerr << "open fail\n"; return -1; }

    float fx = 1818.37f, fy = 1822.49f, cx = 751.06f, cy = 530.44f;
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);
    cv::Mat dist_coeffs = cv::Mat::zeros(5,1,CV_64F);
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(-70,-62.5f,0), cv::Point3f(70,-62.5f,0),
        cv::Point3f(70,62.5f,0),   cv::Point3f(-70,62.5f,0),
    };

    State state = DETECT;
    cv::Point2f track_center;
    float track_dist=0, track_yaw=0, track_pitch=0;
    int lost_count = 0;
    const int MAX_LOST = 15;
    const float MATCH_DIST = 80.0f;

    int threshold_val = 200;
    cv::Mat frame, gray, binary;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, threshold_val, 255, cv::THRESH_BINARY);

        // 灯条检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Lightbar> lightbars;
        for (auto& c : contours) {
            if (cv::contourArea(c) < 10) continue;
            cv::RotatedRect rr = cv::minAreaRect(c);
            float w=rr.size.width, h=rr.size.height;
            float L=std::max(w,h), s=std::min(w,h);
            if (s<3||L<8||L/s<3.0||L/s>15.0) continue;
            float ang=rr.angle; if (w<h) ang+=90;
            if (std::abs(ang-90)>45 && std::abs(ang-90)<135) continue;
            Lightbar lb; lb.rect=rr; lb.center=rr.center;
            lb.length=L; lb.width=s; lb.angle=ang;
            lightbars.push_back(lb);
        }
        std::sort(lightbars.begin(), lightbars.end(),
                  [](const Lightbar&a,const Lightbar&b){return a.center.x<b.center.x;});

        // 配对: 收集所有装甲板(不提前退出)
        std::vector<Armor> armors;
        for (size_t i=0;i<lightbars.size();i++) {
            for (size_t j=i+1;j<lightbars.size();j++) {
                auto&L=lightbars[i]; auto&R=lightbars[j];
                float d=cv::norm(L.center-R.center);
                float dy=std::abs(L.center.y-R.center.y);
                float lr=std::max(L.length,R.length)/std::min(L.length,R.length);
                float dAng=std::abs(L.angle-R.angle);
                if (dAng>15&&dAng<165) continue;
                if (d<10||d>500||dy>30||lr>2.0) continue;
                float armor_ratio = d / ((L.length+R.length)/2.0f);
                if (armor_ratio<0.3f || armor_ratio>3.0f) continue;

                cv::Point2f center=(L.center+R.center)/2.0f;
                float aw=d, ah=(L.length+R.length)/2.0f;
                std::vector<cv::Point2f> ip = {
                    cv::Point2f(center.x-aw/2,center.y-ah/2),
                    cv::Point2f(center.x+aw/2,center.y-ah/2),
                    cv::Point2f(center.x+aw/2,center.y+ah/2),
                    cv::Point2f(center.x-aw/2,center.y+ah/2)};
                cv::Mat rvec,tvec;
                cv::solvePnP(object_points,ip,camera_matrix,dist_coeffs,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
                Armor ar;
                ar.center=center;
                ar.dist = tvec.at<double>(2)/1000.0f;
                ar.yaw  = std::atan2(tvec.at<double>(0),tvec.at<double>(2))*180.0f/CV_PI;
                ar.pitch= std::atan2(tvec.at<double>(1),tvec.at<double>(2))*180.0f/CV_PI;
                armors.push_back(ar);
            }
        }

        // 画所有候选装甲板(白框)
        for (auto& ar : armors) {
            cv::circle(frame, ar.center, 3, cv::Scalar(255,255,255), -1);
            char dbuf[32]; snprintf(dbuf,sizeof(dbuf),"%.1fm",ar.dist);
            cv::putText(frame, dbuf, ar.center+cv::Point2f(5,-5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
        }

        // 选最优装甲板
        Armor best; bool has_best=false;
        if (!armors.empty()) {
            if (state == TRACK) {
                // 跟踪中: 选离 track_center 最近的(保持连续性)
                float best_d=1e9;
                for (auto& ar : armors) {
                    float dd = cv::norm(ar.center - track_center);
                    if (dd < best_d) { best_d=dd; best=ar; has_best=true; }
                }
            } else {
                // 重新搜索: 选距离相机最近的(z最小)
                float best_z=1e9;
                for (auto& ar : armors) {
                    if (ar.dist < best_z) { best_z=ar.dist; best=ar; has_best=true; }
                }
            }
        }

        // 状态机更新
        if (has_best) {
            track_center=best.center; track_dist=best.dist;
            track_yaw=best.yaw; track_pitch=best.pitch;
            state = TRACK;
            lost_count = 0;
        } else {
            lost_count++;
            if (lost_count >= MAX_LOST) state = DETECT;
        }

        // 画跟踪结果
        cv::Scalar color;
        if (state==TRACK) color=cv::Scalar(0,255,0);
        else if (lost_count>0) color=cv::Scalar(0,165,255);
        else color=cv::Scalar(0,0,255);
        cv::circle(frame, track_center, 6, color, -1);
        cv::putText(frame, std::string(STATE_NAME[state])+" lost:"+std::to_string(lost_count)+
                          " candidates:"+std::to_string(armors.size()),
                    cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        char buf[128];
        snprintf(buf,sizeof(buf),"dist:%.2fm yaw:%.1f pitch:%.1f",track_dist,track_yaw,track_pitch);
        cv::putText(frame, buf, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);

        cv::imshow("S6 v2 multi-target", frame);
        cv::imshow("binary", binary);
        if (cv::waitKey(30)=='q') break;
    }
    return 0;
}