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




    #define BOOSTER_TRIGGER_INIT_SPEED 2.0f    //一秒2发的转速
    #define BOOSTER_TRIGGER_PLUG_SPEED 0.3f    //堵转速度 维持.5秒判定结束
    #define BOOSTER_TRIGGER_INIT_COUNT 500     //500ms  

    #define auto_aim_test_mode 1 //自瞄测试模式，1时自瞄状态下手动发弹
    #define auto_trigger_init 0 //自动弹道初始化,1时手动控制，左上往左中
#endif