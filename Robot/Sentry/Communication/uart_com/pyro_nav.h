/**
 * @file uart_comm.h
 * @brief 外部导航 UART 驱动（底盘板侧）
 *
 * 仿照 supercap_drv_t 的「组合（Driver HAS-A Task）」模式：
 * 1. get_instance() 单例
 * 2. 内部 task_base_t 任务（ISR 只搬字节，任务线程校验 + 解包）
 * 3. DMA 发送缓冲
 * 4. FreeRTOS Message Buffer 收包
 */

  #ifndef __PYRO_NAV_H__
  #define __PYRO_NAV_H__

#include "uart_msg.h"
#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"

namespace pyro
{

class nav_t
{
  public:
    /* Public Methods --------------------------------------------------------*/
    /** @brief 单例。 */
    static nav_t *get_instance();

    /** @brief 启动内部任务并使能通信。 */
    void start_rx() const;

    /** @brief 通过 DMA 发送命令数据（MCU → 导航）。 */
    status_t send_cmd(const mcu2nav_data_t &cmd) const; // NOLINT

    /** @brief 获取最新收到的导航数据。 */
    [[nodiscard]] const nav2mcu_data_t &get_rx_msg() const;

    /** @brief 连接状态。 */
    [[nodiscard]] bool check_online() const;

  private:
    /**
     * @brief 构造函数。
     * @param uart_handle 指向现有 PYRO UART 驱动实例的指针。
     */
    explicit nav_t(uart_drv_t *uart_handle);

    /** @brief 析构函数。停止任务并释放资源。 */
    ~nav_t();

    /* Private Task Implementation (Composition) -----------------------------*/
    class nav_task_t final : public task_base_t
    {
      public:
        explicit nav_task_t(nav_t *owner_ptr)
            : task_base_t("nav_task", 128, 128, priority_t::BELOW_NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        nav_t *_owner;
    };

    /* Private Members -------------------------------------------------------*/
    uart_drv_t *_uart_drv;
    nav_task_t *_task;       // 内部任务实例
    mcu2nav_msg_t *_tx_buffer; // DMA 发送缓冲
    MessageBufferHandle_t _rx_msg_buf;

    nav2mcu_data_t _latest_rx{};
    bool _is_online;

    // TODO 用户填实际帧头
    static constexpr uint8_t RX_SOF = 0xA1; // 导航 → MCU
    static constexpr uint8_t TX_SOF = 0xA4; // MCU → 导航

    /* Private Methods (Logic) -----------------------------------------------*/
    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size,
                     BaseType_t &xHigherPriorityTaskWoken) const;

    static status_t error_check(const nav2mcu_msg_t *buf);
    void unpack(const nav2mcu_msg_t *buf);
};

} // namespace pyro

#endif // __UART_COMM_H__
