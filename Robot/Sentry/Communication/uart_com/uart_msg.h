#ifndef __UART_MSG_H__
#define __UART_MSG_H__ 


struct frame_header
{
    uint8_t sof;
};

// 帧尾（CRC16）
struct frame_tailer
{
    uint16_t crc16;
};

// mcu2aim 特有：CRC 之后额外一个回车字节
struct frame_enter
{
    uint8_t enter;
};


struct nav2mcu_data_t {

    float vx;
    float vy;
    float vz;
    float wz;
    uint8_t stuck{};
    float yaw;
    uint8_t in_aim;
    uint8_t mode; // 1为进攻，2为防御，3为移动
    bool scan{};
    bool yaw_align{};
};
struct nav2mcu_msg_t {
    nav2mcu_data_t data;
    frame_header header;
    frame_tailer tailer;
};


struct mcu2nav_data_t {
    uint16_t self_hp;
    uint16_t self_ammo;
    uint8_t game_state;
    uint16_t self_base_hp;
    uint16_t self_outpost_hp;
    uint16_t game_time;
};
struct mcu2nav_msg_t {
    frame_header header;
    mcu2nav_data_t data;
    frame_tailer tailer;
};







#endif
