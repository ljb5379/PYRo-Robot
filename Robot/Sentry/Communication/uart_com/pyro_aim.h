#ifndef __PYRO_AIM_H__
#define __PYRO_AIM_H__ 


#include "uart_msg.h"
#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"
#include "pyro_bsp_uart.h"

namespace pyro
{

class aim_t
{
  public:
    /* Public Methods --------------------------------------------------------*/
    /** @brief 单例。 */
    static aim_t *get_instance();

    /** @brief 启动内部任务并使能通信。 */
    void start_rx() const;

    /** @brief 通过 DMA 发送命令数据（MCU → AIM）。 */
    status_t send_cmd(const mcu2aim_data_t &cmd) const; // NOLINT

    /** @brief 获取最新收到的 AIM 数据。 */
    [[nodiscard]] const aim2mcu_data_t &get_rx_msg() const;

    /** @brief 连接状态。 */
    [[nodiscard]] bool check_online() const;

  private:
    /**
     * @brief 构造函数。
     * @param uart_handle 指向现有 PYRO UART 驱动实例的指针。
     */
    explicit aim_t(uart_drv_t *uart_handle);

    /** @brief 析构函数。停止任务并释放资源。 */
    ~aim_t();

    /* Private Task Implementation (Composition) -----------------------------*/
    class aim_task_t final : public task_base_t
    {
      public:
        explicit aim_task_t(aim_t *owner_ptr)
            : task_base_t("aim_task", 128, 128, priority_t::BELOW_NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        aim_t *_owner;
    };

    /* Private Members -------------------------------------------------------*/
    uart_drv_t *_uart_drv;
    aim_task_t *_task;       // 内部任务实例
    mcu2aim_msg_t *_tx_buffer; // DMA 发送缓冲
    MessageBufferHandle_t _rx_msg_buf;

    aim2mcu_data_t _latest_rx{};
    bool _is_online;

    // TODO 用户填实际帧头
    static constexpr uint8_t RX_SOF = 0xA1; // 导航 → MCU
    static constexpr uint8_t TX_SOF = 0xA4; // MCU → 导航

    /* Private Methods (Logic) -----------------------------------------------*/
    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size,
                     BaseType_t &xHigherPriorityTaskWoken) const;

    static status_t error_check(const aim2mcu_msg_t *buf);
    void unpack(const aim2mcu_msg_t *buf);
};









};



#endif
