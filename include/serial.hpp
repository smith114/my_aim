// serial.hpp: USB CDC 组帧/解帧/打开
#pragma once
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// 小端写
static void put_i16(std::vector<uint8_t>&b, int16_t v){b.push_back(v&0xff);b.push_back((v>>8)&0xff);}
static void put_f32(std::vector<uint8_t>&b, float v){uint8_t*p=(uint8_t*)&v;for(int i=0;i<4;i++)b.push_back(p[i]);}
static void put_i32(std::vector<uint8_t>&b, int32_t v){uint8_t*p=(uint8_t*)&v;for(int i=0;i<4;i++)b.push_back(p[i]);}

// 组上位机->下位机 18 字节帧
inline std::vector<uint8_t> pack_send(float yaw, float pitch, int32_t fire_flag) {
    std::vector<uint8_t> b;
    put_i16(b, 16);            // body_len
    put_i16(b, 1);             // ID1
    put_f32(b, yaw);
    put_f32(b, pitch);
    put_i16(b, 2);             // ID2
    put_i32(b, fire_flag);
    return b;
}

// 打开 USB CDC 串口
inline int open_serial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
    cfmakeraw(&tty);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}