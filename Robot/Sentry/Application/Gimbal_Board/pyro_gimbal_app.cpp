#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

#include "pyro_sentry_gimbal.h"

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_can_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_board_comm.h"
#include "pyro_bsp_uart.h"

#include "pyro_aim.h"


using namespace pyro;

constexpr uint32_t EVENT_BIT_SPINNING = (1 << 0);

static TaskHandle_t gimbal_task_handle                        = nullptr;
static pyro::sentry_gimbal_t *sentry_gimbal_ptr                = nullptr;
static pyro::sentry_gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
static pyro::sentry_gimbal_deps_t *gimbal_deps_ptr             = nullptr;

aim2mcu_data_t aim2gimbal_msg;



extern "C" {
// void aim2chassis_msg()
// {
//     aim2gimbal_msg = aim_t::get_instance()->get_rx_msg();

// }

void deps_init()
{
    gimbal_deps_ptr = new pyro::sentry_gimbal_deps_t();

    gimbal_deps_ptr->motor_deps.motor_yaw = new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                    pyro::bsp_can::can1);
    gimbal_deps_ptr->motor_deps.motor_pitch = new pyro::dm_motor_drv_t(0x1, 0x0, bsp_can::can2);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_position_range(-PI , PI);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_rotate_range(-20, 20);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_torque_range(-10, 10);

    gimbal_deps_ptr->pitch_max_rad     = -0.11f; // 最高的时候
    gimbal_deps_ptr->pitch_min_rad     = -0.34f; // 最低的时候
    gimbal_deps_ptr->yaw_max_rad       = 0.70f;
    gimbal_deps_ptr->yaw_min_rad       = -0.70f;

    gimbal_deps_ptr->pid_deps.yaw_pos_pid = new pyro::pid_t(20.50f, 0.0f, 0.0f,0,10);
    gimbal_deps_ptr->pid_deps.yaw_spd_pid = new pyro::pid_t(0.13f, 0.0f, 0.0f,0,10.0f);

    gimbal_deps_ptr->pid_deps.pitch_pos_pid = new pyro::pid_t(30.0f, 0.1f, 0.0f,1.0f,45);
    gimbal_deps_ptr->pid_deps.pitch_spd_pid = new pyro::pid_t(2.0f, 0.2f, 0.00f,3.0f,12);
}


void gimbal_aim2mcu()//可以不用 直接用gimbal_dr16andaim2cmd()
{   if(aim_t::get_instance()->check_online() == false)
    {
        aim2gimbal_msg.aim_state = 0;
        aim2gimbal_msg.fire = 0;
        aim2gimbal_msg.is_single_shot = 0;
        aim2gimbal_msg.aim_state = 0;
        return;
    }
    aim2gimbal_msg = pyro::aim_t::get_instance()->get_rx_msg();
    
}

// void aim_rx_init()
// {
//     auto &uart1 = pyro::bsp_uart::get_uart1();
//     uart1.add_rx_event_callback([aim2gimbal_msg]{})

// }

//   云台接收
void gimbal_dr162cmd()                                      //_______纯手动， 无自瞄
{       

    if(pyro::dr16_drv_t::instance().check_online() == false)
    {
        gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->delta_pitch = 0;
        gimbal_cmd_ptr->delta_yaw   = 0;
        gimbal_cmd_ptr->auto_mode = false;
        return;
    }

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->delta_pitch = 0;
        gimbal_cmd_ptr->delta_yaw   = 0;
        gimbal_cmd_ptr->auto_mode = false;
        return;
    }
    gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    gimbal_cmd_ptr->delta_pitch = -vrc.axes.ry * 0.005f;
    gimbal_cmd_ptr->delta_yaw   =1* (-vrc.axes.rx * 0.008f);
    gimbal_cmd_ptr->auto_mode = false;

    
}
void gimbal_dr16andaim2cmd()                                //  自瞄 + 遥控开关
{
    if(aim_t::get_instance()->check_online() == false)
    {
        gimbal_cmd_ptr->aim_mode = 0;
        gimbal_cmd_ptr->target_yaw = 0;
        gimbal_cmd_ptr->target_pitch = 0;
        gimbal_cmd_ptr->auto_mode = false;
        return;
    }

    aim2gimbal_msg = aim_t::get_instance()->get_rx_msg();
    

//_______________dr16控制总开关
    if(pyro::dr16_drv_t::instance().check_online() == false)
    {
        gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->auto_mode = false;
        return;
    }

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();
    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->auto_mode = false;
        return;
    }
    gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;

    gimbal_cmd_ptr->auto_mode = true;

//____________________自瞄自主控制_______________________//
    bool aim_open = false;
    if(vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN){aim_open = true;}
    if(aim_open){
    gimbal_cmd_ptr->aim_mode = aim2gimbal_msg.aim_state;
    gimbal_cmd_ptr->target_yaw = aim2gimbal_msg.shoot_yaw;
    gimbal_cmd_ptr->target_pitch = aim2gimbal_msg.shoot_pitch;
    gimbal_cmd_ptr->auto_mode = true;

    }

}


//  云台转发
//暂时不需要管自瞄有没有开，自瞄没跟底盘通信
void gimbal_dr16andnav2chassis_cmd(uint32_t notify_value)   //   遥控 + 导航
{
    g2c_msg_t msg{};
    
    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static int8_t delta_yaw = 0;
    static bool active      = false;
    static bool follow_en   = true;
    static bool spinning    = false;
    static bool nav_en   = false;


    if(pyro::dr16_drv_t::instance().check_online() == false)
    {
        active              = 0;
        vx                  = 0;
        vy                  = 0;
        wz                  = 0;
        delta_yaw           = 0;
        follow_en           = false;
        spinning            = false;
        nav_en              = false;

        msg.vx        = 0;
        msg.vy        = 0;
        msg.delta_yaw = 0;
        msg.flags     = 0;

        pyro::board_comm_t::instance().send(msg);

        return;

    }

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        active              = 0;
        vx                  = 0;
        vy                  = 0;
        wz                  = 0;
        delta_yaw           = 0;
        follow_en           = false;
        spinning            = false;
        nav_en              = false;

        msg.vx        = 0;
        msg.vy        = 0;
        msg.delta_yaw = 0;
        msg.flags     = 0;

        pyro::board_comm_t::instance().send(msg);

        return;
    }

    active              = 1;
    vx     = static_cast<int8_t>(-(vrc.axes.ly) * 127);
    vy     = static_cast<int8_t>(vrc.axes.lx * 127);
    wz     = 0;
    delta_yaw = static_cast<int8_t>(vrc.axes.rx * 127);

    if(abs(vx) < 5)vx = 0;
    if(abs(vy) < 5)vy = 0;
    if(abs(wz) < 5)wz = 0;
    if(abs(delta_yaw) < 5)delta_yaw = 0;

    static uint16_t spinning_count = 0;

    // if(pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos){
    //     if(spinning_count > 1000)
    //     {
    //         spinning = true;
    //         follow_en = false; //依赖底盘写法  谨慎调整！！！ 目前默认开启跟随
    //     }
    //     else {spinning_count++;}
    // }

    // if(EVENT_BIT_SPINNING & notify_value)
    // {
    //     spinning = false;
    //     spinning_count = 0;
    //     //follow_en =  !follow_en;
    //     follow_en = true;
    //     nav_en = !nav_en;
    // }
    if(pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos){
        
            spinning = false;
            follow_en = true; //依赖底盘写法  谨慎调整！！！ 目前默认开启跟随

            nav_en = true;
    }
    if(pyro::sw_pos_t::MID == vrc.switches.right.current_pos){

            spinning = false;
            follow_en = true; //依赖底盘写法  谨慎调整！！！ 目前默认开启跟随
            nav_en = false;

    }



    msg.vx        = vx;
    msg.vy        = vy;
    msg.delta_yaw = delta_yaw;
    msg.flags     = static_cast<uint8_t>((active<<0)|(follow_en<<1)|(spinning<<2)|(nav_en<<3));

    pyro::board_comm_t::instance().send(msg);
}

void gimbal_dr162chassis_cmd(uint32_t notify_value)         // 手动// 右下小陀螺
{
    g2c_msg_t msg{};
    
    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static int8_t delta_yaw = 0;
    static bool active      = false;
    static bool follow_en   = true;
    static bool spinning    = false;
    static bool nav_en   = false;


    if(pyro::dr16_drv_t::instance().check_online() == false)
    {
        active              = 0;
        vx                  = 0;
        vy                  = 0;
        wz                  = 0;
        delta_yaw           = 0;
        follow_en           = false;
        spinning            = false;
        nav_en              = false;

        msg.vx        = 0;
        msg.vy        = 0;
        msg.delta_yaw = 0;
        msg.flags     = 0;

        pyro::board_comm_t::instance().send(msg);

        return;

    }

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        active              = 0;
        vx                  = 0;
        vy                  = 0;
        wz                  = 0;
        delta_yaw           = 0;
        follow_en           = false;
        spinning            = false;
        nav_en              = false;

        msg.vx        = 0;
        msg.vy        = 0;
        msg.delta_yaw = 0;
        msg.flags     = 0;

        pyro::board_comm_t::instance().send(msg);

        return;
    }

    active              = 1;
    vx     = static_cast<int8_t>(-(vrc.axes.ly) * 127);
    vy     = static_cast<int8_t>(vrc.axes.lx * 127);
    wz     = 0;
    delta_yaw = static_cast<int8_t>(vrc.axes.rx * 127);

    if(abs(vx) < 5)vx = 0;
    if(abs(vy) < 5)vy = 0;
    if(abs(wz) < 5)wz = 0;
    if(abs(delta_yaw) < 5)delta_yaw = 0;

    static uint16_t spinning_count = 0;

    if(pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos){
        if(spinning_count > 1000)
        {
            spinning = true;
            follow_en = false; //依赖底盘写法  谨慎调整！！！ 目前默认开启跟随
        }
        else {spinning_count++;}
    }

    if(EVENT_BIT_SPINNING & notify_value)
    {
        spinning = false;
        spinning_count = 0;
        follow_en =  !follow_en;
   
    }



    msg.vx        = vx;
    msg.vy        = vy;
    msg.delta_yaw = delta_yaw;
    msg.flags     = static_cast<uint8_t>((active<<0)|(follow_en<<1)|(spinning<<2)|(nav_en<<3));

    pyro::board_comm_t::instance().send(msg);
}



void gimbal_mcu2aim_data(){
    mcu2aim_data_t mcu2aim_msg;
    sentry_gimbal_context_t _ctx = sentry_gimbal_ptr->get_ctx();

        float yaw, pitch, roll;
        ins_drv_t *ins = ins_drv_t::get_instance();
        ins->get_angles_b(&yaw, &pitch, &roll);
        yaw                               = yaw / 180 * PI;
        pitch                             = pitch / 180 * PI;

        mcu2aim_msg.curr_yaw         = yaw;
        mcu2aim_msg.curr_pitch       = pitch;
        mcu2aim_msg.self_v_magnitude = 0;
        mcu2aim_msg.self_v_angle     = 0;
        mcu2aim_msg.curr_speed       = 0;
        mcu2aim_msg.shoot_delay      = 0;
        mcu2aim_msg.state            = _ctx.cmd->aim_mode;
        mcu2aim_msg.stop_record      = 0;
        mcu2aim_msg.autoaim          = _ctx.cmd->auto_mode;
        mcu2aim_msg.enemy_color      = 1;       
        pyro::aim_t::get_instance()->send_cmd(mcu2aim_msg);



}
void sentry_gimbal_thread(void *argument)
{
    while (true)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

        //gimbal_aim2mcu();

        //aim2chassis_msg();

        
        // 云台接收指令 手动在代码中开关
         gimbal_dr162cmd();
         //gimbal_dr16andaim2cmd(); 

         //云台转发指令
        //gimbal_dr162chassis_cmd(notify_val);
        gimbal_dr16andnav2chassis_cmd(notify_val);

        

        sentry_gimbal_ptr->set_command(*gimbal_cmd_ptr);
        vTaskDelay(1);
    }
}

void sentry_gimbal_init(void)
{
    gimbal_cmd_ptr     = new pyro::sentry_gimbal_cmd_t();
    sentry_gimbal_ptr = pyro::sentry_gimbal_t::instance();

    deps_init();
    sentry_gimbal_ptr->configure(*gimbal_deps_ptr);
    sentry_gimbal_ptr->start();

    xTaskCreate(sentry_gimbal_thread, "start_sentry_gimbal_thread", 256,
                nullptr, configMAX_PRIORITIES - 1, &gimbal_task_handle);
    auto &vrc = pyro::rc_drv_t::read();
    pyro::sw_broker::subscribe(&vrc.switches.right, pyro::sw_event_t::DOWN_TO_MID, gimbal_task_handle, EVENT_BIT_SPINNING);
    vTaskDelete(nullptr);


    
}

}
