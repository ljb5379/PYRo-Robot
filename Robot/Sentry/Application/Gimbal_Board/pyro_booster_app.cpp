#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_bsp_can.h"
#include "pyro_can_drv.h"

#include "pyro_aim.h"
#include "uart_msg.h"
#include "pyro_board_comm.h"
using namespace pyro;

//已处理 
//1. trigger角度获取优化（待验证/function1_enable宏定义开关/ 反馈还使用没算减速比的速度）
//2. 摩擦轮下力时刹车
//3. 

//右上下力
//右中手动
//右下自动
//上电后左摇杆初始放左中



//1.校准    上力后第一次开摩擦润前左上往左下              //自瞄模式上力就校准  //阻塞，除下力外其他操作都无效
//单发前先判断是否校准，多发不用判断，但连发后主动校准到单发位置（先将校准标志位清零，连发结束）  无论自瞄或手控，

//2.单发    左上往左中开摩擦轮，左下往      //自瞄模式发is_single_shoot=1, aim2booster_msg.fire为1时 发1发弹，若在校准没用
//          左中单发，若没校准则没用        
//3.多发    左下600ms                     //自瞄模式 is_single_shoot=0, aim2booster_msg.fire为1时 发多发弹，若在校准没用       booster_cmd_ptr->multi_shoot从0到1变化时校准标志位重置     

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_ON = (1 << 0);
constexpr uint32_t EVENT_BIT_SINGGLE_SHOOT   = (1 << 1);
constexpr uint32_t EVENT_BIT_FIRE = (1 << 2); 
constexpr uint32_t EVENT_BIT_FRIC_OFF = (1 << 3); 

static TaskHandle_t booster_task_handle                          = nullptr;
static pyro::sentry_booster_t *sentry_booster_ptr                = nullptr;
static pyro::sentry_booster_cmd_t *booster_cmd_ptr               = nullptr;
static pyro::sentry_booster_deps_t *booster_deps_ptr             = nullptr;

static pyro::aim2mcu_data_t aim2booster_msg;


extern "C" {
static void deps_init();
static void booster_dr162cmd(uint32_t notify_value);
static void chassis2booster_rx();

static void chassis2booster_rx()
{
    c2g_msg_t boostermsg;
    if (pyro::board_comm_t::instance().read(boostermsg))
    {
        // 4字节预留数据，格式待定
        booster_cmd_ptr->shoot_speed = boostermsg.shoot_speed;
        booster_cmd_ptr->data1 = boostermsg.data2;
        booster_cmd_ptr->data2 = boostermsg.data3;
    }
#if BOARD_COMM_TIMEOUT_C2G_ENABLE
    else if (pyro::board_comm_t::instance().is_stale<c2g_msg_t>(BOARD_COMM_TIMEOUT_C2G_MS))
    {
        // 底盘失联：数据清零
        booster_cmd_ptr->shoot_speed = 0;
        booster_cmd_ptr->data1 = 0;
        booster_cmd_ptr->data2 = 0;
    }
#endif
}
void deps_init()
    {
        booster_deps_ptr = new pyro::sentry_booster_deps_t();

        
        booster_deps_ptr->motor_deps.fric_left = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::bsp_can::can1); 
        booster_deps_ptr->motor_deps.fric_right = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::bsp_can::can1);
        booster_deps_ptr->motor_deps.trigger = new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::bsp_can::can1);





        booster_deps_ptr->pid_deps.l_fric_pid = 
                                new pyro::pid_t(0.05f, 0.0f, 0.0f, 0.8f, 20.0f);
        booster_deps_ptr->pid_deps.r_fric_pid =  
                                new pyro::pid_t(0.05f, 0.0f, 0.0f, 0.8f, 20.0f);
        booster_deps_ptr->pid_deps.trig_pos_pid = 
                                new pyro::pid_t(250.0f     -1  , 1.0f, 0.00f, 0.00f, 250.0f);
        booster_deps_ptr->pid_deps.trig_spd_pid = 
                                new pyro::pid_t(0.065f  , 0.05f, 0.0002f, 0.09f, 20.0f);

    }
    


void booster_dr162cmd(uint32_t notify_value)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static uint32_t multi_count = 0;

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        booster_cmd_ptr->fric_on = 0;
        booster_cmd_ptr->fire_count   = 0;
        booster_cmd_ptr->real_hit_speed   = 0;
        booster_cmd_ptr->multi_shoot   =  0;
        booster_cmd_ptr->start_trigger_init = 0;
        
        return;
    }
    
    else{
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
        
    //1.开关摩擦轮
        if(notify_value & EVENT_BIT_FRIC_ON)
        {   //左上拨左中
            booster_cmd_ptr->fric_on                = true;
        }
        if(notify_value & EVENT_BIT_FRIC_OFF)
        {   //左中拨左上
            booster_cmd_ptr->fric_on                = false;
        }   
    //2.单发/多发
        booster_cmd_ptr->multi_shoot        =  0;  //默认单发
    if(booster_cmd_ptr->fric_on){
        if(pyro::sw_pos_t::DOWN == vrc.switches.left.current_pos){
            
            
            if(multi_count > 600){
            booster_cmd_ptr->multi_shoot          =  true;
            }
            else{
                
                multi_count ++;
            }
        }
        if(notify_value & EVENT_BIT_FIRE)
        {   
            //booster_cmd_ptr->singgle_shoot          =  !booster_cmd_ptr->singgle_shoot;
            booster_cmd_ptr->multi_shoot          =  false;
            multi_count = 0;
            
        }
        

        if(notify_value & EVENT_BIT_SINGGLE_SHOOT)
        {   
            //booster_cmd_ptr->singgle_shoot          =  !booster_cmd_ptr->singgle_shoot;
            
            booster_cmd_ptr->fire_count ++;
            booster_cmd_ptr->multi_shoot        =  0;
        }
    }


    }

}
void booster_dr16andaim2cmd(uint32_t notify_value){
    
    
    if(aim_t::get_instance()->check_online() == false)
    {
        booster_cmd_ptr->fric_on = false;
        booster_cmd_ptr->multi_shoot = false;
        booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;

        return;
    }

    aim2booster_msg = aim_t::get_instance()->get_rx_msg();
    

//_______________dr16控制总开关
    if(pyro::dr16_drv_t::instance().check_online() == false)
    {
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        booster_cmd_ptr->fric_on           = false;
        return;
    }

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();
    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        booster_cmd_ptr->fric_on = false;
        return;
    }
    booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;

    
//____________________自瞄自主控制_______________________//

    if(vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN){
    //booster_cmd_ptr->fric_on = aim2booster_msg.fire; //自瞄锁到人了;
    booster_cmd_ptr->fric_on = true;
    //booster_cmd_ptr->multi_shoot = false;
    booster_cmd_ptr->multi_shoot = !aim2booster_msg.is_single_shot;

    //booster_cmd_ptr->fire_count += aim2booster_msg.fire;

    if( !booster_cmd_ptr->multi_shoot ){
        #if _auto_shoot == 0
        if(notify_value & EVENT_BIT_SINGGLE_SHOOT)
        {   
            
            
            booster_cmd_ptr->fire_count ++;
            
        }
        #elif auto_shoot == 1
        if(aim2booster_msg.fire)
        {   
    
            booster_cmd_ptr->fire_count ++;
        }   
        #endif
    }
    }

}



    void sentry_booster_thread(void *argument)
    {
        while (true)
        {   
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);


            //chassis_rxcmd();
            // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
            //booster_dr162cmd(notify_val);
            // booster_dr162chassis_cmd();
            
            chassis2booster_rx();
            #if AUTOAIM_MODE ==1
            booster_dr16andaim2cmd(notify_val);
            #elif AUTOAIM_MODE ==0
            booster_dr162cmd(notify_val);
            #endif
            sentry_booster_ptr->set_command(*booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void sentry_booster_init(void)
    {
        //========版间通信订阅========
        //pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can3, 0x103);
        

        
        booster_cmd_ptr     = new pyro::sentry_booster_cmd_t();
        sentry_booster_ptr = pyro::sentry_booster_t::instance();

        

        deps_init();
        sentry_booster_ptr->configure(*booster_deps_ptr);
        sentry_booster_ptr->start();


        xTaskCreate(sentry_booster_thread, "start_sentry_booster_thread", 256,
                    nullptr, configMAX_PRIORITIES - 1, &booster_task_handle);
            // --- DR16 拨杆绑定 ---
        read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID, booster_task_handle, EVENT_BIT_FRIC_ON);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID, booster_task_handle, EVENT_BIT_SINGGLE_SHOOT);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_DOWN, booster_task_handle, EVENT_BIT_FIRE);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_UP, booster_task_handle, EVENT_BIT_FRIC_OFF);


        vTaskDelete(nullptr);
    }






}
