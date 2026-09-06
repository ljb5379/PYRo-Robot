#ifndef __UART_MSG_H__
#define __UART_MSG_H__

#include <cstdint>

namespace pyro
{

#pragma pack(push, 1)

// 帧头（1 字节 SOF，靠它区分消息类型）
struct frame_header
{
    uint8_t sof;
};

// 帧尾（CRC16）
struct frame_tailer
{
    uint16_t crc16;
};

// 预留：mcu2aim 特有，CRC 之后额外一个回车字节
struct frame_enter
{
    uint8_t enter;
};

// ============================================================================
// 导航数据段
// ============================================================================

// 外部导航 → MCU：底盘运动/状态指令
struct nav2mcu_data_t
{
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

// MCU → 外部导航：比赛/状态信息
struct mcu2nav_data_t
{
    uint16_t self_hp;
    uint16_t self_ammo;
    uint8_t game_state;
    uint16_t self_base_hp;
    uint16_t self_outpost_hp;
    uint16_t game_time;
};

// ============================================================================
// 整帧结构体（SOF 必须在首字节）
// ============================================================================

// 收：导航 → MCU  = [SOF][nav2mcu_data_t][CRC16]
struct nav2mcu_msg_t
{
    frame_header header;
    nav2mcu_data_t data;
    frame_tailer tailer;
};

// 发：MCU → 导航 = [SOF][mcu2nav_data_t][CRC16]
struct mcu2nav_msg_t
{
    frame_header header;
    mcu2nav_data_t data;
    frame_tailer tailer;
};

#pragma pack(pop)

// ============================================================================
// 编译期校验：数据段字节数 + 整帧长度
// ============================================================================
static_assert(sizeof(nav2mcu_data_t) == 25, "nav2mcu_data_t must be 25 bytes");
static_assert(sizeof(mcu2nav_data_t) == 11, "mcu2nav_data_t must be 11 bytes");
static_assert(sizeof(nav2mcu_msg_t) == 28, "nav2mcu_msg_t must be 28 bytes"); // 1 + 25 + 2
static_assert(sizeof(mcu2nav_msg_t) == 14, "mcu2nav_msg_t must be 14 bytes"); // 1 + 11 + 2

} // namespace pyro

#endif // __UART_MSG_H__
