#include "pyro_core_def.h"
#ifndef __BOOSTER_CONFIG_H__
#define __BOOSTER_CONFIG_H__ 


#define BOOSTER_SHOOT_FRIC_LINE_V    24.0f
#define BOOSTER_SHOOT_DELTA_V        0.2f
#define BOOSTER_SHOOT_MAX_SPEED      24.5f
#define BOOSTER_SHOOT_MIN_SPEED      23.0f

#define BOOSTER_FRIC_R              0.034f
#define BOOSTER_SHOOT_FRIC_RADPS    BOOSTER_SHOOT_FRIC_LINE_V/BOOSTER_FRIC_R
#define BOOSTER_SHOOT_TRIG_SPEED 10.0f      //一秒多少发弹
#define BOOSTER_TRIGGER_RATE 36
#define BOOSTER_ONE_SHOOT_ANGLE PI/4

#define new_function1_enable  1    //新波胆盘展开逻辑
#define new_function2_enable  1    //摩擦轮下力刹车


    #define BOOSTER_TRIGGER_INIT_SPEED 2.0f    //一秒2发的转速
    #define BOOSTER_TRIGGER_PLUG_SPEED 0.5f    //堵转速度 维持.5秒判定结束
    #define BOOSTER_TRIGGER_INIT_COUNT 500     //500ms  

#endif
