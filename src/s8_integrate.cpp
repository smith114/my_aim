// S8 final: 整合链路 + S6状态机跟踪
// 相机->灯条->配对->PnP->状态机选优->组帧->串口
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cmath>

static void put_i16(std::vector<uint8_t>&b,int16_t v){b.push_back(v&0xff);b.push_back((v>>8)&0xff);}
static void put_f32(std::vector<uint8_t>&b,float v){uint8_t*p=(uint8_t*)&v;for(int i=0;i<4;i++)b.push_back(p[i]);}
static void put_i32(std::vector<uint8_t>&b,int32_t v){uint8_t*p=(uint8_t*)&v;for(int i=0;i<4;i++)b.push_back(p[i]);}
static std::vector<uint8_t> pack_send(float yaw,float pitch,int32_t fire){
    std::vector<uint8_t> b;
    put_i16(b,16); put_i16(b,1); put_f32(b,yaw); put_f32(b,pitch);
    put_i16(b,2); put_i32(b,fire);
    return b;
}
static int open_serial(const char* port){
    int fd=open(port,O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0) return -1;
    struct termios tty;
    if(tcgetattr(fd,&tty)!=0){close(fd);return -1;}
    cfmakeraw(&tty);
    tty.c_cc[VMIN]=0; tty.c_cc[VTIME]=1;
    tcsetattr(fd,TCSANOW,&tty);
    return fd;
}
struct Lightbar{ cv::RotatedRect rect; cv::Point2f center; float length,width,angle; };
struct Armor{ cv::Point2f center; float dist,yaw,pitch; };

enum State{DETECT,TRACK,LOST};

int main(int argc, char** argv){
    cv::VideoCapture cap;
    if(argc<2) cap.open(0); else cap.open(argv[1]);
    if(!cap.isOpened()){std::cerr<<"cannot open video\n";return -1;}

    int serial_fd=open_serial("/dev/ttyACM0");
    if(serial_fd<0) serial_fd=open_serial("/dev/ttyACM1");
    bool mock=(serial_fd<0);
    if(mock) std::cout<<"[MOCK] no /dev/ttyACM*\n";
    else     std::cout<<"[SERIAL] fd="<<serial_fd<<"\n";

    float fx=1818.37f,fy=1822.49f,cx=751.06f,cy=530.44f;
    cv::Mat K=(cv::Mat_<double>(3,3)<<fx,0,cx, 0,fy,cy, 0,0,1);
    cv::Mat dist=cv::Mat::zeros(5,1,CV_64F);
    std::vector<cv::Point3f> obj={
        cv::Point3f(-70,-62.5f,0),cv::Point3f(70,-62.5f,0),
        cv::Point3f(70,62.5f,0),cv::Point3f(-70,62.5f,0)};

    // 状态机
    State state=DETECT;
    cv::Point2f track_c;
    float track_dist=0,track_yaw=0,track_pitch=0;
    int lost=0;
    const int MAX_LOST=15;

    int threshold_val=200;
    cv::Mat frame,gray,binary;
    while(true){
        cap>>frame;
        if(frame.empty()) break;
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
        cv::threshold(gray,binary,threshold_val,255,cv::THRESH_BINARY);

        // 灯条
        std::vector<std::vector<cv::Point>> cs;
        cv::findContours(binary,cs,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        std::vector<Lightbar> lbs;
        for(auto&c:cs){
            if(cv::contourArea(c)<10) continue;
            cv::RotatedRect rr=cv::minAreaRect(c);
            float w=rr.size.width,h=rr.size.height;
            float L=std::max(w,h),s=std::min(w,h);
            if(s<3||L<8||L/s<3||L/s>15) continue;
            float ang=rr.angle; if(w<h) ang+=90;
            if(std::abs(ang-90)>45 && std::abs(ang-90)<135) continue;
            Lightbar lb; lb.rect=rr; lb.center=rr.center;
            lb.length=L; lb.width=s; lb.angle=ang;
            lbs.push_back(lb);
        }
        std::sort(lbs.begin(),lbs.end(),[](const Lightbar&a,const Lightbar&b){return a.center.x<b.center.x;});

        // 配对收集所有装甲板
        std::vector<Armor> armors;
        for(size_t i=0;i<lbs.size();i++){
            for(size_t j=i+1;j<lbs.size();j++){
                auto&L=lbs[i]; auto&R=lbs[j];
                float d=cv::norm(L.center-R.center);
                float dy=std::abs(L.center.y-R.center.y);
                float lr=std::max(L.length,R.length)/std::min(L.length,R.length);
                float dAng=std::abs(L.angle-R.angle);
                if(dAng>15&&dAng<165) continue;
                if(d<10||d>500||dy>30||lr>2.0) continue;
                float ar=d/((L.length+R.length)/2.0f);
                if(ar<0.3f||ar>3.0f) continue;
                cv::Point2f c=(L.center+R.center)/2.0f;
                float aw=d, ah=(L.length+R.length)/2.0f;
                std::vector<cv::Point2f> ip={
                    cv::Point2f(c.x-aw/2,c.y-ah/2),cv::Point2f(c.x+aw/2,c.y-ah/2),
                    cv::Point2f(c.x+aw/2,c.y+ah/2),cv::Point2f(c.x-aw/2,c.y+ah/2)};
                cv::Mat rvec,tvec;
                cv::solvePnP(obj,ip,K,dist,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
                Armor ar2; ar2.center=c;
                ar2.dist=tvec.at<double>(2)/1000.0f;
                ar2.yaw=std::atan2(tvec.at<double>(0),tvec.at<double>(2))*180.0/CV_PI;
                ar2.pitch=std::atan2(tvec.at<double>(1),tvec.at<double>(2))*180.0/CV_PI;
                armors.push_back(ar2);
            }
        }

        // 选最优
        bool has=false; Armor best;
        if(!armors.empty()){
            if(state==TRACK){
                float bd=1e9;
                for(auto&a:armors){float dd=cv::norm(a.center-track_c); if(dd<bd){bd=dd;best=a;has=true;}}
            }else{
                float bz=1e9;
                for(auto&a:armors){if(a.dist<bz){bz=a.dist;best=a;has=true;}}
            }
        }

        // 状态机更新
        if(has){
            track_c=best.center; track_dist=best.dist;
            track_yaw=best.yaw; track_pitch=best.pitch;
            state=TRACK; lost=0;
        }else{
            lost++;
            if(lost>=MAX_LOST) state=DETECT;
            // 保持 track_* 上一帧值
        }

        // 发送(用平滑跟踪值)
        auto pkt=pack_send(track_yaw, track_pitch, 0);
        if(mock){
            printf("yaw=%6.2f pitch=%6.2f state=%d lost=%d n=%d | ",
                   track_yaw,track_pitch,state,lost,(int)armors.size());
            for(auto b:pkt) printf("%02x ",b);
            printf("\n");
        }else{
            write(serial_fd,pkt.data(),pkt.size());
        }

        cv::Scalar color = state==TRACK ? cv::Scalar(0,255,0) :
                           lost>0 ? cv::Scalar(0,165,255) : cv::Scalar(0,0,255);
        if(state==TRACK||lost>0) cv::circle(frame,track_c,6,color,-1);
        char buf[128];
        snprintf(buf,sizeof(buf),"dist:%.2f yaw:%.1f pitch:%.1f lost:%d",
                 track_dist,track_yaw,track_pitch,lost);
        cv::putText(frame,buf,cv::Point(10,30),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,255),2);
        cv::imshow("S8 final",frame);
        if(cv::waitKey(30)=='q') break;
    }
    if(serial_fd>=0) close(serial_fd);
    return 0;
}