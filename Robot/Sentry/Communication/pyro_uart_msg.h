#ifndef PYRO_UART_MSG_H
#define PYRO_UART_MSG_H

#include "pyro_bsp_uart.h"
#include <cstdint>

namespace pyro
{

// 消息方向标记（供 uart_comm_t 判断该消息是收还是发）
constexpr uint8_t UART_DIR_RX = 0; // 只收（外部 → MCU）
constexpr uint8_t UART_DIR_TX = 1; // 只发（MCU → 外部）

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

// mcu2aim 特有：CRC 之后额外一个回车字节
struct frame_enter
{
    uint8_t enter;
};

// ============================================================================
// 数据段结构体（uart_comm_t 的 read<T>/send<T> 均以这些 data 结构体为 T）
// 元数据说明：
//   SOF       帧头字节（TODO 用户填实际值）
//   uart()    所在串口（默认 UART7，可逐消息改）
//   DIR       方向（UART_DIR_RX 只收 / UART_DIR_TX 只发）
//   HAS_ENTER 是否带末尾回车字节
//   ENTER     回车字节值（仅 HAS_ENTER=true 时有效）
//   FRAME_LEN 整帧长度 = SOF(1) + data(N) + CRC16(2) [+ enter(1)]
// ============================================================================

// 外部 → MCU：导航数据
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

    static constexpr uint8_t  SOF       = 0xA1; // TODO 用户填实际帧头
    static constexpr uint8_t  DIR       = UART_DIR_RX;
    static constexpr bool     HAS_ENTER = false;
    static constexpr uint16_t FRAME_LEN = 28; // 1 + 25 + 2

    static uart_drv_t& uart() { return bsp_uart::get_uart7(); }
};

// 外部 → MCU：自瞄数据
struct aim2mcu_data_t
{
    float shoot_yaw;
    float shoot_yaw_speed;
    float shoot_yaw_acceleration;
    float shoot_pitch;
    float shoot_pitch_speed;
    float shoot_pitch_acceleration;
    uint8_t fire           : 1;
    uint8_t is_single_shot : 1;
    uint8_t target_id      : 6;
    uint8_t aim_state;

    static constexpr uint8_t  SOF       = 0xA2; // TODO 用户填实际帧头
    static constexpr uint8_t  DIR       = UART_DIR_RX;
    static constexpr bool     HAS_ENTER = false;
    static constexpr uint16_t FRAME_LEN = 29; // 1 + 26 + 2

    static uart_drv_t& uart() { return bsp_uart::get_uart7(); }
};

// MCU → 外部：云台当前姿态/速度反馈
struct mcu2aim_data_t
{
    float curr_yaw;
    float curr_pitch;
    float self_v_magnitude;
    float self_v_angle;
    float curr_speed;
    uint8_t shoot_delay;
    uint8_t state       : 5;
    uint8_t stop_record : 1;
    uint8_t autoaim     : 1;
    uint8_t enemy_color : 1;

    static constexpr uint8_t  SOF       = 0xA3; // TODO 用户填实际帧头
    static constexpr uint8_t  DIR       = UART_DIR_TX;
    static constexpr bool     HAS_ENTER = true;
    static constexpr uint8_t  ENTER     = 0x0A; // TODO 用户填实际回车字节
    static constexpr uint16_t FRAME_LEN = 26;   // 1 + 22 + 2 + 1

    static uart_drv_t& uart() { return bsp_uart::get_uart7(); }
};

// MCU → 外部：比赛/状态信息
struct mcu2nav_data_t
{
    uint16_t self_hp;
    uint16_t self_ammo;
    uint8_t game_state;
    uint16_t self_base_hp;
    uint16_t self_outpost_hp;
    uint16_t game_time;

    static constexpr uint8_t  SOF       = 0xA4; // TODO 用户填实际帧头
    static constexpr uint8_t  DIR       = UART_DIR_TX;
    static constexpr bool     HAS_ENTER = false;
    static constexpr uint16_t FRAME_LEN = 14; // 1 + 11 + 2

    static uart_drv_t& uart() { return bsp_uart::get_uart7(); }
};

// ============================================================================
// 完整帧结构体（用于 sizeof 校验帧长；框架内收发缓冲用）
// ============================================================================
struct nav2mcu_msg_t
{
    frame_header header;
    nav2mcu_data_t data;
    frame_tailer tailer;
};

struct aim2mcu_msg_t
{
    frame_header header;
    aim2mcu_data_t data;
    frame_tailer tailer;
};

struct mcu2aim_msg_t
{
    frame_header header;
    mcu2aim_data_t data;
    frame_tailer tailer;
    frame_enter enter;
};

struct mcu2nav_msg_t
{
    frame_header header;
    mcu2nav_data_t data;
    frame_tailer tailer;
};

#pragma pack(pop)

// 握手协议（当前空占位，流程待定）
struct handshake_protocol_t
{
};

// ============================================================================
// 编译期校验：数据段字节数 + FRAME_LEN 与整帧 sizeof 一致性
// ============================================================================
static_assert(sizeof(nav2mcu_data_t) == 25, "nav2mcu_data_t must be 25 bytes");
static_assert(sizeof(aim2mcu_data_t) == 26, "aim2mcu_data_t must be 26 bytes");
static_assert(sizeof(mcu2aim_data_t) == 22, "mcu2aim_data_t must be 22 bytes");
static_assert(sizeof(mcu2nav_data_t) == 11, "mcu2nav_data_t must be 11 bytes");

static_assert(nav2mcu_data_t::FRAME_LEN == sizeof(nav2mcu_msg_t), "nav2mcu FRAME_LEN mismatch");
static_assert(aim2mcu_data_t::FRAME_LEN == sizeof(aim2mcu_msg_t), "aim2mcu FRAME_LEN mismatch");
static_assert(mcu2aim_data_t::FRAME_LEN == sizeof(mcu2aim_msg_t), "mcu2aim FRAME_LEN mismatch");
static_assert(mcu2nav_data_t::FRAME_LEN == sizeof(mcu2nav_msg_t), "mcu2nav FRAME_LEN mismatch");

} // namespace pyro

#endif
