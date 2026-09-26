#include "pyro_sentry_gimbal.h"
#include "gimbal_config.h"
#include "pyro_algo_common.h"
#include <arm_math.h>

#include "pyro_jcom.h"
#include "pyro_bsp_uart.h"




#define JCOM_DEBUG_PORT PYRO_UART1
float target_pitch1;
float target_pitch2;
float current_pitch1;

float pitchspdtarget;
float pitchspdcurrent;

namespace pyro{

    jcom_drv_t jcom_drv(15,&JCOM_DEBUG_PORT);


    float low_pass_filter(float input, float *prev_output, float alpha)
{

    *prev_output = alpha * input + (1.0f - alpha) * (*prev_output);
    return *prev_output;
}

    sentry_gimbal_t::sentry_gimbal_t():module_base_t("sentry_gimbal"){
        _ctx={};
    }
;
    status_t sentry_gimbal_t::_init(){
        _ctx.motor=_module_deps.motor_deps;
        _ctx.pid=_module_deps.pid_deps;

        _ctx.data.gimbal_pitch_offset_rad= GIMBAL_PITCH_OFFSET_RAD;
        _ctx.data.gimbal_yaw_offset_rad= GIMBAL_YAW_OFFSET_RAD;

        _ctx.data.yaw_max_rad = _module_deps.yaw_max_rad;
        _ctx.data.yaw_min_rad = _module_deps.yaw_min_rad;
        _ctx.data.pitch_max_rad = _module_deps.pitch_max_rad;
        _ctx.data.pitch_min_rad = _module_deps.pitch_min_rad;

        return PYRO_OK;
    }

    void sentry_gimbal_t::_update_feedback(){
        _ctx.motor.motor_pitch->update_feedback();
        _ctx.motor.motor_yaw->update_feedback();

        _ctx.data.current_data.pitch_pos=
                    loop_fp32_constrain(_ctx.motor.motor_pitch->get_current_position()
                                            -_ctx.data.gimbal_pitch_offset_rad,-PI,PI);
        _ctx.data.current_data.yaw_pos=
                    loop_fp32_constrain( _ctx.motor.motor_yaw->get_current_position()
                                            -_ctx.data.gimbal_yaw_offset_rad,-PI,PI);

        _ctx.data.current_data.pitch_spd=
                    _ctx.motor.motor_pitch->get_current_rotate();
        _ctx.data.current_data.yaw_spd=
                    _ctx.motor.motor_yaw->get_current_rotate();
        
        _ctx.data.current_data.pitch_torque=
                    _ctx.motor.motor_pitch->get_current_torque();
        _ctx.data.current_data.yaw_torque=
                    _ctx.motor.motor_yaw->get_current_torque();

        

    //    2. 读取 IMU 数据作为底盘姿态反馈
    float raw_yaw, raw_pitch, raw_roll;
    ins_drv_t::get_instance()->get_rads_n(&raw_yaw, &raw_pitch, &raw_roll);


    raw_pitch -= PITCH_OFFSET_RAD;
    raw_roll  -= ROLL_OFFSET_RAD;


    // --- 一阶低通滤波 (LPF) ---
    // 为了快速验证，这里使用 static 变量保存上一次的滤波状态
    // 如果确认有效，建议将它们移到 _ctx.data 结构体中
    static float filtered_pitch = 0.0f;
    static float filtered_roll  = 0.0f;
    static float filtered_yaw   = 0.0f;
    static bool  is_first_run   = true;

    // 滤波系数 alpha：(0, 1]
    // alpha = 1.0 表示完全不滤波；alpha 越小，抗噪声能力越强，但相位延迟越大。
    // 对于 500Hz~1000Hz 的控制循环，0.1f ~ 0.3f 通常是一个比较理想的甜点值。
    const float LPF_ALPHA = 0.15f;

    if (is_first_run)
    {
        filtered_pitch = raw_pitch;
        filtered_roll  = raw_roll;
        filtered_yaw   = raw_yaw;
        is_first_run   = false;
    }
    else
    {
        low_pass_filter(raw_pitch, &filtered_pitch, LPF_ALPHA);
        low_pass_filter(raw_roll,  &filtered_roll,  LPF_ALPHA);
        low_pass_filter(raw_yaw,   &filtered_yaw,   LPF_ALPHA);
    }
    _ctx.data.imu_data.current_pitch_rad = filtered_pitch;
    _ctx.data.imu_data.current_roll_rad  = filtered_roll;    
    _ctx.data.imu_data.current_yaw_rad = filtered_yaw;
    
    
    //---暂不滤波---
    // _ctx.imu_data.current_pitch_rad = raw_pitch;
    // _ctx.imu_data.current_roll_rad  = raw_roll;    
    // _ctx.imu_data.current_yaw_rad = raw_yaw;
    }

void sentry_gimbal_t::_solve(){

    //先对模式进行判断 若自瞄模式则为给定数值（暂定）， 若手动控制则为增量

    if(_ctx.cmd->auto_mode == true)
    {
        _ctx.data.target_data.pitch_pos = _ctx.cmd->target_pitch;
        _ctx.data.target_data.yaw_pos = _ctx.cmd->target_yaw;

    }
    else{
    if(_ctx.cmd->delta_pitch<0.00005f && _ctx.cmd->delta_pitch>-0.00005f)
        {_ctx.cmd->delta_pitch=0.0f;}
    if(_ctx.cmd->delta_yaw<0.00005f && _ctx.cmd->delta_yaw>-0.00005f)
        {_ctx.cmd->delta_yaw=0.0f;}


    _ctx.data.target_data.pitch_pos  += _ctx.cmd->delta_pitch;
    _ctx.data.target_data.yaw_pos    += _ctx.cmd->delta_yaw;
    }


//统一进行限位
    if(_ctx.data.target_data.pitch_pos>_ctx.data.pitch_max_rad)
        {_ctx.data.target_data.pitch_pos=_ctx.data.pitch_max_rad;}
    if(_ctx.data.target_data.pitch_pos<_ctx.data.pitch_min_rad)
        {_ctx.data.target_data.pitch_pos=_ctx.data.pitch_min_rad;}

    if( _ctx.data.target_data.yaw_pos>_ctx.data.yaw_max_rad)
        { _ctx.data.target_data.yaw_pos=_ctx.data.yaw_max_rad;}
    if( _ctx.data.target_data.yaw_pos<_ctx.data.yaw_min_rad)
        { _ctx.data.target_data.yaw_pos=_ctx.data.yaw_min_rad;}

//jcom_drv.add_data()

}

void sentry_gimbal_t::_gimbal_control(){
    

    float pitch_error=loop_fp32_constrain
                    (_ctx.data.target_data.pitch_pos 
                -   _ctx.data.current_data.pitch_pos,
                    -PI,PI);

    float pitch_pid_pos_out =_ctx.pid.pitch_pos_pid->calculate(pitch_error,0);

    

//low_pass_filter(pitch_pid_pos_out, &_ctx.data.target_data.pitch_spd, 0.1f);


    static float flitter_gravaty_error_angle = -0.40f - _ctx.data.current_data.pitch_pos;
    float gravaty_error_angle = -0.40f - _ctx.data.current_data.pitch_pos;

    low_pass_filter(gravaty_error_angle, &flitter_gravaty_error_angle, 0.4f);

    // _ctx.data.out_data.pitch_torque=  _ctx.pid.pitch_spd_pid
    //                     ->calculate(pitch_pid_pos_out*0 + _ctx.cmd->delta_pitch*1000,
    //                                 _ctx.data.current_data.pitch_spd)
    //                     -0.6 * cos(flitter_gravaty_error_angle); 
low_pass_filter(   _ctx.pid.pitch_spd_pid
                         ->calculate(pitch_pid_pos_out ,
                                     _ctx.data.current_data.pitch_spd)
                         -0.64 * cos(flitter_gravaty_error_angle),&_ctx.data.out_data.pitch_torque,0.1f );


target_pitch1 = _ctx.data.target_data.pitch_pos;
target_pitch2 =-0.4f - flitter_gravaty_error_angle;
current_pitch1 = _ctx.data.current_data.pitch_pos;
                    
pitchspdtarget =  pitch_pid_pos_out;
pitchspdcurrent = _ctx.data.current_data.pitch_spd;

    float yaw_error=loop_fp32_constrain
                    (_ctx.data.target_data.yaw_pos 
                -   _ctx.data.current_data.yaw_pos,
                    -PI,PI);

                    

    //float yaw_pid_pos_out=_ctx.pid.yaw_pos_pid->calculate(yaw_error,0);
    float yaw_pid_pos_out=_ctx.pid.yaw_pos_pid->calculate( _ctx.cmd->delta_yaw*85 , _ctx.data.current_data.yaw_pos);
    

    _ctx.data.out_data.yaw_torque=_ctx.pid.yaw_spd_pid
                        ->calculate(yaw_pid_pos_out,
                                    _ctx.data.current_data.yaw_spd);
    
}

void sentry_gimbal_t::_send_motor_command() const{
    _ctx.motor.motor_pitch->send_torque(_ctx.data.out_data.pitch_torque*0);
    _ctx.motor.motor_yaw->send_torque(_ctx.data.out_data.yaw_torque);

}
void sentry_gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

};
