#include "uart_comm.h"
#include "uart_msg.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"

namespace pyro {
nav_t::nav_t(uart_drv_t* uart) : _uart(uart), _rx_msg_buf(nullptr), tx_msg(nullptr), rx_msg(nullptr) {

        _rx_msg_buf = xMessageBufferCreate(sizeof(nav2mcu_msg_t));

        rx_msg = static_cast<nav2mcu_msg_t *>(pvPortDmaMalloc(sizeof(nav2mcu_msg_t)));
        tx_msg = static_cast<mcu2nav_msg_t *>(pvPortDmaMalloc(sizeof(mcu2nav_msg_t)));

    }

nav_t::~nav_t() {
    if (_rx_msg_buf) {
        vMessageBufferDelete(_rx_msg_buf);
    }
    if (rx_msg) {
        vPortFree(rx_msg);
    }
    if (tx_msg) {
        vPortFree(tx_msg);
    }
    
    if (_uart){ 
        _uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
        _uart = nullptr;
    }
}
nav_t* nav_t::instance(){
    static nav_t instance(&PYRO_UART7);
    return &instance;
}

void nav_t::init() {
    if (_uart) {
        _uart->add_rx_event_callback(
            [this](uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken) -> bool {
                return this->rx_calback(p, size, xHigherPriorityTaskWoken);
            },
            reinterpret_cast<uint32_t>(this));
    }


}

bool nav_t::rx_calback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken) {
    bool check = true;
    
    // if(size == sizeof(nav2mcu_msg_t)){
        
    // }
    if(check ==  true){
        xMessageBufferSendFromISR(_rx_msg_buf, p, sizeof(nav2mcu_msg_t), &xHigherPriorityTaskWoken);
    }
    return true;

}

nav2mcu_msg_t* nav_t::get_rx_msg(){
    rx_msg = static_cast<nav2mcu_msg_t *>(xMessageBufferReceiveFromISR(_rx_msg_buf, rx_msg, sizeof(nav2mcu_msg_t), nullptr));


}



}