#ifndef __UART_COMM_H__
#define __UART_COMM_H__ 

#include "pyro_bsp_uart.h"
#include "pyro_uart_msg.h"
#include "message_buffer.h"
namespace pyro { 

class nav_t { 
public:



    nav_t(uart_drv_t* uart);
    
    static nav_t* instance();
    

    void init();

private:
    ~nav_t();

    bool rx_calback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken); // ISR

    nav2mcu_msg_t* get_rx_msg();


    

    nav2mcu_msg_t* rx_msg;
    mcu2nav_msg_t* tx_msg;
    uart_drv_t* _uart;
    MessageBufferHandle_t _rx_msg_buf;
    uint32_t _cmd_id = 0x12345678; // Unique identifier for this instance
};



}
#endif
