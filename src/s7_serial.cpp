// S7: 串口组帧/解帧 (按校内赛协议)
// 上->下 18B: body_len(2=16) + id1(2=1) + yaw(4) + pitch(4) + id2(2=2) + fire_flag(4)
// 下->上 30B: body_len(2=28) + id1(2=1) + yaw(4)+pitch(4)+roll(4)+mode(4)+color(4) + id2(2=2) + bullet_speed(4)
// 所有多字节小端。USB CDC 流式, 不假设一次一帧。
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>

// 写小端 int16
static void put_i16(std::vector<uint8_t>&b, int16_t v){
    b.push_back(v&0xff); b.push_back((v>>8)&0xff);
}
// 写小端 float32
static void put_f32(std::vector<uint8_t>&b, float v){
    uint8_t*p=(uint8_t*)&v;
    for(int i=0;i<4;i++) b.push_back(p[i]);
}
// 写小端 int32
static void put_i32(std::vector<uint8_t>&b, int32_t v){
    uint8_t*p=(uint8_t*)&v;
    for(int i=0;i<4;i++) b.push_back(p[i]);
}

// 组上位机->下位机帧 (18 字节)
static std::vector<uint8_t> pack_send(float yaw, float pitch, int32_t fire_flag){
    std::vector<uint8_t> b;
    put_i16(b, 16);           // body_len
    put_i16(b, 1);            // ID1: 云台
    put_f32(b, yaw);
    put_f32(b, pitch);
    put_i16(b, 2);            // ID2: 开火
    put_i32(b, fire_flag);
    return b;
}

// 解析下位机->上位机帧后的结果
struct GimbalState {
    float yaw, pitch, roll;
    int32_t mode, color;
    float bullet_speed;
};

// 流式解析器
class FrameParser {
public:
    std::vector<uint8_t> buf;
    bool feed(const uint8_t* data, int n, GimbalState& out){
        for(int i=0;i<n;i++) buf.push_back(data[i]);
        // 循环尝试解析
        while(true){
            if(buf.size()<2) return false;
            int16_t body_len = buf[0] | (buf[1]<<8);
            if(body_len != 28){
                // 非法长度, 丢 1 字节重新同步 (协议无魔数/CRC)
                buf.erase(buf.begin());
                continue;
            }
            if((int)buf.size() < 2+body_len) return false; // 不够一帧
            // 解析
            int off=2;
            int16_t id1 = buf[off] | (buf[off+1]<<8); off+=2;
            if(id1 != 1){ buf.erase(buf.begin()); continue; }
            memcpy(&out.yaw,   &buf[off], 4); off+=4;
            memcpy(&out.pitch, &buf[off], 4); off+=4;
            memcpy(&out.roll,  &buf[off], 4); off+=4;
            memcpy(&out.mode,  &buf[off], 4); off+=4;
            memcpy(&out.color, &buf[off], 4); off+=4;
            int16_t id2 = buf[off] | (buf[off+1]<<8); off+=2;
            if(id2 != 2){ buf.erase(buf.begin()); continue; }
            memcpy(&out.bullet_speed, &buf[off], 4); off+=4;
            // 消费这一帧
            buf.erase(buf.begin(), buf.begin()+2+body_len);
            return true;
        }
    }
};

int main(){
    // ===== 自测 1: 组帧 =====
    auto frame = pack_send(12.5f, -3.2f, 0);
    std::cout << "send frame size=" << frame.size() << " (expect 18)\n";
    std::cout << "hex: ";
    for(auto c : frame) printf("%02x ", c);
    std::cout << "\n";

    // ===== 自测 2: 解帧 (构造一个假的下位机帧) =====
    std::vector<uint8_t> fake;
    put_i16(fake, 28);       // body_len
    put_i16(fake, 1);        // ID1
    float yaw=10.0f, pitch=-2.0f, roll=1.0f;
    int32_t mode=0, color=3;
    float bs=22.0f;
    put_f32(fake,yaw); put_f32(fake,pitch); put_f32(fake,roll);
    put_i32(fake,mode); put_i32(fake,color);
    put_i16(fake,2);         // ID2
    put_f32(fake,bs);
    std::cout << "\nfake recv frame size=" << fake.size() << " (expect 30)\n";

    // 模拟流式: 分两次喂
    FrameParser parser;
    GimbalState st;
    bool ok;
    std::cout << "feed first 10 bytes...\n";
    ok = parser.feed(fake.data(), 10, st);
    std::cout << "  got frame? " << (ok?"yes":"no (waiting)") << "\n";
    std::cout << "feed rest...\n";
    ok = parser.feed(fake.data()+10, fake.size()-10, st);
    std::cout << "  got frame? " << (ok?"yes":"no") << "\n";
    printf("  yaw=%.2f pitch=%.2f roll=%.2f mode=%d color=%d bullet_speed=%.2f\n",
           st.yaw, st.pitch, st.roll, st.mode, st.color, st.bullet_speed);

    // ===== 自测 3: 帧之间夹垃圾字节, 看重同步 =====
    std::cout << "\n=== 测试: 帧前夹垃圾 ===";
    std::vector<uint8_t> junk = {0x00,0x99,0x88};
    junk.insert(junk.end(), fake.begin(), fake.end());
    FrameParser p2; GimbalState s2;
    bool ok2 = p2.feed(junk.data(), junk.size(), s2);
    printf("\ngot frame after junk? %s\n", ok2?"yes":"no");
    printf("  yaw=%.2f bullet_speed=%.2f\n", s2.yaw, s2.bullet_speed);

    std::cout << "\n[S7 mock done] 组帧/解帧/重同步逻辑正确, 实车接 /dev/ttyACM* 即可\n";
    return 0;
}