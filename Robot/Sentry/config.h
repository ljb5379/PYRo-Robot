#ifndef PYRO_ROBOT_CONFIG_H
#define PYRO_ROBOT_CONFIG_H

#include "pyro_core_config.h"

 #define DR16_UART PYRO_UART5

 #if BOARD == CHASSIS_BOARD
      #define NAV_UART PYRO_UART10
 #endif
 #if BOARD == GIMBAL_BOARD
    #define AUTOAIM_UART PYRO_UART7
 #endif


//调试时使用，开关自瞄和导航。暂时不同时开
#define AUTOAIM_MODE 0
#define NAV_MODE 1

#define s_yaw_enable 0 //小yaw是否能操控

#define new_function3_enable 1

#define rad 40
#endif
