#ifndef PYRO_UART_COMM_H
#define PYRO_UART_COMM_H

#include "pyro_uart_msg.h"
#include "pyro_core_def.h"
#include "pyro_crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace pyro
{

// 串口流式接收的字节流缓冲大小（ISR 写入、解析线程读出）
constexpr size_t UART_COMM_STREAM_SIZE = 512;
// 解析线程单帧临时缓冲上限（大于最大整帧长 29 即可）
constexpr uint16_t UART_COMM_MAX_FRAME = 40;
// 阻塞发送的 HAL 超时（ms）
constexpr uint32_t UART_COMM_TX_TIMEOUT_MS = 10;
// RX 回调注册时的 owner ID
constexpr uint32_t UART_COMM_OWNER = 0x55415254u; // "UART"

/**
 * @brief 每个 RX 消息类型一个"订阅槽"：解码缓存 + 新鲜标志。
 *        按类型静态唯一（get() 返回同一个实例）。UART 没有 can_msg_buffer_t，
 *        底层换成「字节缓存 + 新鲜标志」，解析线程解码后直接写入。
 */
template <typename T>
class uart_rx_slot_t
{
public:
    static uart_rx_slot_t& get()
    {
        static uart_rx_slot_t instance;
        return instance;
    }

    /** @brief 解析线程解码完成后写入（跳过 SOF，指向 data 段，拷贝 sizeof(T) 字节）。 */
    void notify(const uint8_t* data)
    {
        taskENTER_CRITICAL();
        std::memcpy(&_cache, data, sizeof(T));
        _fresh   = true;
        _last_rx = xTaskGetTickCount();
        taskEXIT_CRITICAL();
    }

    /** @brief 消费式读取最新值；返回自上次读取后是否有更新。 */
    bool read(T& out)
    {
        if (!_fresh)
            return false;

        taskENTER_CRITICAL();
        out    = _cache;
        _fresh = false;
        taskEXIT_CRITICAL();
        return true;
    }

    /** @brief 距上次收到数据是否已超时（失联判定）。从未收到过数据也视为超时。 */
    bool is_stale(TickType_t timeout_ms) const
    {
        return (xTaskGetTickCount() - _last_rx) >= pdMS_TO_TICKS(timeout_ms);
    }

private:
    uart_rx_slot_t() = default;

    T _cache{};
    volatile bool _fresh  = false;
    TickType_t _last_rx   = 0;
};

/**
 * @brief 外部 UART 通信管理类（单例）。
 *        负责注册 RX 回调、起解析线程（ISR 只搬运字节、任务线程流式分帧 + CRC 校验），
 *        对外提供类型化的 send<T> / read<T> / is_stale<T> 接口（与 board_comm_t 风格一致）。
 */
class uart_comm_t
{
public:
    static uart_comm_t& instance()
    {
        static uart_comm_t instance;
        return instance;
    }

    /** @brief 建字节流缓冲 + 注册 RX 回调（串口接收使能前调用）。 */
    status_t init();

    /** @brief 创建解析任务线程。 */
    status_t start();

    /** @brief 读取接口（消费式；T 需为 RX 消息的 data 结构体）。 */
    template <typename T>
    bool read(T& out) const
    {
        return uart_rx_slot_t<T>::get().read(out);
    }

    /** @brief 失联判定。 */
    template <typename T>
    bool is_stale(TickType_t timeout_ms) const
    {
        return uart_rx_slot_t<T>::get().is_stale(timeout_ms);
    }

    /** @brief 发送接口（组帧 SOF + data + CRC16 → 阻塞写）。T 需为 TX 消息的 data 结构体。 */
    template <typename T>
    status_t send(const T& msg) const;

private:
    uart_comm_t() = default;

    static bool rx_callback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken); // ISR
    static void parse_thread(void* arg);

    MessageBufferHandle_t _rx_stream{};
};

// ---- 发送实现（inline）：组帧 [SOF][data][CRC16]（mcu2aim 追加 enter）→ 阻塞写 ----
// 注：用阻塞 write 而非非阻塞 DMA，因为栈上 frame 在函数返回后失效，DMA 会访问悬垂指针。
template <typename T>
status_t uart_comm_t::send(const T& msg) const
{
    static_assert(T::DIR == UART_DIR_TX, "T is not a TX message");

    uint8_t frame[1 + sizeof(T) + 2 + (T::HAS_ENTER ? 1 : 0)];
    frame[0] = T::SOF;
    std::memcpy(&frame[1], &msg, sizeof(T));

    uint16_t len = 1 + sizeof(T) + 2;
    append_crc16_check_sum(frame, len); // CRC 覆盖 SOF + data

    if constexpr (T::HAS_ENTER)
    {
        frame[len] = T::ENTER;
        len += 1;
    }

    return T::uart().write(frame, len, UART_COMM_TX_TIMEOUT_MS);
}

} // namespace pyro

#endif
