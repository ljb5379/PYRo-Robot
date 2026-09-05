#include "pyro_uart_comm.h"

namespace pyro
{

status_t uart_comm_t::init()
{
    if (_rx_stream == nullptr)
    {
        _rx_stream = xMessageBufferCreate(UART_COMM_STREAM_SIZE);
        if (_rx_stream == nullptr)
            return PYRO_ERROR;
    }

    // v1：所有消息默认走 UART7，只注册一个回调；将来分口时需按口分组注册。
    nav2mcu_data_t::uart().add_rx_event_callback(
        [](uint8_t* p, uint16_t size, BaseType_t& woken) -> bool {
            return uart_comm_t::rx_callback(p, size, woken);
        },
        UART_COMM_OWNER);

    return PYRO_OK;
}

status_t uart_comm_t::start()
{
    const BaseType_t ret = xTaskCreate(parse_thread, "pyro_uart_comm", 256, nullptr,
                                       configMAX_PRIORITIES - 1, nullptr);
    return (ret == pdPASS) ? PYRO_OK : PYRO_ERROR;
}

// ISR 回调：只把空闲收到的整段字节塞进字节流，绝不在中断里做解析。
bool uart_comm_t::rx_callback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken)
{
    uart_comm_t& self = uart_comm_t::instance();

    if (self._rx_stream != nullptr)
    {
        // 流满时返回 0（丢弃），仍继续消费 DMA 缓冲，保证接收不中断。
        xMessageBufferSendFromISR(self._rx_stream, p, size, &xHigherPriorityTaskWoken);
    }

    return true; // 总是消费并切换缓冲
}

// 解析线程：从字节流逐字节读，跑流式状态机，处理半包/粘包 + CRC 校验。
void uart_comm_t::parse_thread(void* arg)
{
    (void)arg;
    uart_comm_t& self = uart_comm_t::instance();

    uint8_t frame[UART_COMM_MAX_FRAME]{};
    enum state_t { WAIT_SOF, RECV } state = WAIT_SOF;
    uint16_t expect_len = 0;
    uint16_t idx        = 0;
    uint8_t  type       = 0; // 0 = nav2mcu, 1 = aim2mcu

    while (true)
    {
        uint8_t b;
        if (xMessageBufferReceive(self._rx_stream, &b, 1, portMAX_DELAY) != 1)
            continue;

        if (state == WAIT_SOF)
        {
            if (b == nav2mcu_data_t::SOF)
            {
                type       = 0;
                expect_len = nav2mcu_data_t::FRAME_LEN;
                frame[0]   = b;
                idx        = 1;
                state      = RECV;
            }
            else if (b == aim2mcu_data_t::SOF)
            {
                type       = 1;
                expect_len = aim2mcu_data_t::FRAME_LEN;
                frame[0]   = b;
                idx        = 1;
                state      = RECV;
            }
            // 其余字节：非 SOF，丢弃继续等帧头
        }
        else // RECV
        {
            frame[idx++] = b;
            if (idx == expect_len)
            {
                // 收齐一帧：CRC 覆盖 SOF + data；失败则丢弃并回 WAIT_SOF 重新同步
                if (verify_crc16_check_sum(frame, expect_len))
                {
                    if (type == 0)
                        uart_rx_slot_t<nav2mcu_data_t>::get().notify(frame + 1);
                    else
                        uart_rx_slot_t<aim2mcu_data_t>::get().notify(frame + 1);
                }
                state = WAIT_SOF;
            }
        }
    }
}

} // namespace pyro
