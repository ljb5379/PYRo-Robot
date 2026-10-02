#include "pyro_aim.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include <cstring>

namespace pyro
{

/* ========================================================================== */
/* Inner Task Implementation                                                  */
/* ========================================================================== */

status_t aim_t::aim_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void aim_t::aim_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

/* ========================================================================== */
/* Driver Implementation                                                      */
/* ========================================================================== */

/* instance ------------------------------------------------------------------*/
aim_t *aim_t::get_instance()
{
    static aim_t instance(&PYRO_UART7); // TODO 用户填实际导航串口
    return &instance;
}

/* Constructor & Destructor --------------------------------------------------*/
aim_t::aim_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _is_online(false)
{
    // 注：_latest_rx 已由成员声明处的 {} 值初始化，无需 memset

    // 1. 分配 DMA 可用的发送缓冲
    constexpr size_t buf_size = sizeof(mcu2aim_msg_t);
    _tx_buffer = static_cast<mcu2aim_msg_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer)
    {
        memset(_tx_buffer, 0, buf_size);
    }

    // 2. 实例化内部任务（尚未启动）
    _task = new aim_task_t(this);
}

aim_t::~aim_t()
{
    // 1. 停止并删除任务
    if (_task)
    {
        _task->stop(); // 确保 FreeRTOS 任务被删除
        delete _task;
        _task = nullptr;
    }

    // 2. 注销 UART 回调
    if (_uart_drv)
    {
        _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    }

    // 3. 释放资源
    if (_tx_buffer)
    {
        vPortDmaFree(_tx_buffer);
        _tx_buffer = nullptr;
    }

    if (_rx_msg_buf)
    {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
}

/* Public Control Methods ----------------------------------------------------*/
void aim_t::start_rx() const
{
    // 资源有效时才启动
    if (_task && _uart_drv && _tx_buffer)
    {
        _task->start();
    }
}

/* Logic Implementation (Private) --------------------------------------------*/

// 由 aim_task_t::init() 在任务上下文中调用
void aim_t::init_impl()
{
    // 1. 创建 Message Buffer（约 4 帧 + 开销）
    if (_rx_msg_buf == nullptr)
    {
        _rx_msg_buf = xMessageBufferCreate(sizeof(aim2mcu_msg_t) * 4);
    }

    if (_rx_msg_buf == nullptr)
        return;

    // 2. 注册 RX ISR 回调
    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, const uint16_t size,
               BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

// 由 aim_task_t::run_loop() 调用
void aim_t::run_loop_impl()
{
    while (true)
    {
        static aim2mcu_msg_t pkt;
        static size_t xReceivedBytes;
        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                  400) == sizeof(aim2mcu_msg_t))
        {
            // 收到首帧，判定在线
            _is_online = true;
        }

        while (_is_online)
        {
            // 阻塞等待数据（120 ticks 超时）
            xReceivedBytes =
                xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), 120);

            if (xReceivedBytes == sizeof(aim2mcu_msg_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);
                }
            }
            else if (xReceivedBytes == 0)
            {
                // 超时 -> 离线
                _is_online = false;
            }
        }
    }
}

/* ISR Callback --------------------------------------------------------------*/
bool aim_t::rx_callback(const uint8_t *p_data, const uint16_t size,
                        BaseType_t &xHigherPriorityTaskWoken) const
{
    // ISR 最小校验：帧起始 + 整帧长度 + 帧尾回车
    if (size == sizeof(aim2mcu_msg_t) && p_data[0] == RX_SOF &&
        p_data[sizeof(aim2mcu_msg_t) - 1] == RX_ENTER)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(aim2mcu_msg_t),
                                  &xHigherPriorityTaskWoken);
        return true; // 数据已消费，驱动应切换缓冲
    }
    return false;
}

/* Protocol Helpers ----------------------------------------------------------*/
status_t aim_t::error_check(const aim2mcu_msg_t *buf)
{
    // CRC16 校验（覆盖 SOF + data，不含帧尾回车）
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf),
                                sizeof(aim2mcu_msg_t) - 1))
    {
        return PYRO_ERROR;
    }

    return PYRO_OK;
}

void aim_t::unpack(const aim2mcu_msg_t *buf)
{
    memcpy(&_latest_rx, &buf->data, sizeof(aim2mcu_data_t));
}

/* Transmission --------------------------------------------------------------*/
status_t aim_t::send_cmd(const mcu2aim_data_t &cmd) const
{
    if (!_tx_buffer || !_uart_drv)
        return PYRO_ERROR;

    // 1. 填充帧头
    _tx_buffer->header.sof = TX_SOF;

    // 2. 拷贝载荷
    memcpy(&_tx_buffer->data, &cmd, sizeof(mcu2aim_data_t));

    // 3. 填充帧尾回车
    _tx_buffer->enter.enter = TX_ENTER;

    // 4. 填充 CRC16（覆盖 SOF + data，不含帧尾回车）
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer),
                           sizeof(mcu2aim_msg_t) - 1);

    // 5. DMA 写
    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer),
                            sizeof(mcu2aim_msg_t));
}

/* Getters -------------------------------------------------------------------*/
const aim2mcu_data_t &aim_t::get_rx_msg() const
{
    return _latest_rx;
}

bool aim_t::check_online() const
{
    return _is_online;
}

} // namespace pyro
