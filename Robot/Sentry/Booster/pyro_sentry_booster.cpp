#include "pyro_sentry_booster.h"

#include "booster_config.h"
#include "pyro_algo_common.h"
#include <arm_math.h>

namespace pyro{


    sentry_booster_t::sentry_booster_t():module_base_t("sentry_booster"){
        _ctx={};
    }
;
    status_t sentry_booster_t::_init(){
        _ctx.motor=_module_deps.motor_deps;
        _ctx.pid=_module_deps.pid_deps;


        return PYRO_OK;
    }

    void sentry_booster_t::_update_feedback(){
        _ctx.motor.fric_left->update_feedback();
        _ctx.motor.fric_right->update_feedback();
        _ctx.motor.trigger->update_feedback();

        static float last_rotate = 0.0f;
        static float last_position = 0.0f;

        _ctx.data.current_data.left_spd=
                    _ctx.motor.fric_left->get_current_rotate();
        _ctx.data.current_data.right_spd=
                    _ctx.motor.fric_right->get_current_rotate();
        
        _ctx.data.current_data.left_torque=
                    _ctx.motor.fric_left->get_current_torque();
        _ctx.data.current_data.right_torque=
                    _ctx.motor.fric_right->get_current_torque();

        _ctx.data.current_data.trigger_spd=
                    _ctx.motor.trigger->get_current_rotate();
        _ctx.data.current_data.trigger_torque=
                    _ctx.motor.trigger->get_current_torque();
        _ctx.data.current_data.trigger_pos=
                    _ctx.motor.trigger->get_current_position();

   

        //获取处理后角度 rad
        float _error_angle =  loop_fp32_constrain(_ctx.data.current_data.trigger_pos - last_position,-PI, PI);
        _ctx.data.current_data.trigger_equal_pos  += _error_angle / BOOSTER_TRIGGER_RATE;
        _ctx.data.current_data.trigger_equal_pos = loop_fp32_constrain(_ctx.data.current_data.trigger_equal_pos,-PI, PI);
        last_position = _ctx.data.current_data.trigger_pos;

        //获取处理后角速度rad/s
        _ctx.data.current_data.trigger_equal_spd = _ctx.data.current_data.trigger_spd / BOOSTER_TRIGGER_RATE;  

    }


void sentry_booster_t::_fric_control(){

    _ctx.data.fric_ready = false;
    if(_ctx.cmd->fric_on){
        #if auto_trigger_init==0
            if(_ctx.data.first_init == true){
                _ctx.data.init_start_flag = true;
                _ctx.data.first_init = false;
                return;
            }
            #endif
        static float fric_spd = BOOSTER_SHOOT_FRIC_LINE_V;
            
        // _ctx.data.target_data.left_spd =       BOOSTER_SHOOT_FRIC_RADPS;//有裁判系统后速度换成计算值
        // _ctx.data.target_data.right_spd =    - BOOSTER_SHOOT_FRIC_RADPS;
        if(_ctx.cmd->shoot_speed>1e-5){
            if(_ctx.cmd->shoot_speed>BOOSTER_SHOOT_MAX_SPEED){
                // _ctx.data.target_data.left_spd -= BOOSTER_SHOOT_DELTA_V;
                // _ctx.data.target_data.right_spd += BOOSTER_SHOOT_DELTA_V;
                fric_spd -= BOOSTER_SHOOT_DELTA_V;
            }
            else if(_ctx.cmd->shoot_speed<BOOSTER_SHOOT_MIN_SPEED){
                // _ctx.data.target_data.left_spd += BOOSTER_SHOOT_DELTA_V;
                // _ctx.data.target_data.right_spd -= BOOSTER_SHOOT_DELTA_V;
                fric_spd += BOOSTER_SHOOT_DELTA_V;
            }
            else{}

            if(fric_spd>BOOSTER_SHOOT_FRIC_LINE_V*1.3f){
                fric_spd = BOOSTER_SHOOT_FRIC_LINE_V*1.3f;
            }
            else if(fric_spd<BOOSTER_SHOOT_FRIC_LINE_V*0.7f){
                fric_spd = BOOSTER_SHOOT_FRIC_LINE_V*0.7f;
            }
            else{}
        }
            _ctx.data.target_data.left_spd =       fric_spd/BOOSTER_FRIC_R;
            _ctx.data.target_data.right_spd =    - fric_spd/BOOSTER_FRIC_R ;

            if(        fabs(_ctx.data.target_data.left_spd-_ctx.data.current_data.left_spd)<1.0f
                    && fabs(_ctx.data.target_data.right_spd-_ctx.data.current_data.right_spd)<1.0f){
                _ctx.data.fric_ready = true;
            }

    }
    else{
        _ctx.data.target_data.left_spd   =       0.0f;
        _ctx.data.target_data.right_spd  =       0.0f;
    }
    // _ctx.target_data.left_spd=_ctx.cmd->fric_target_speed;
    // _ctx.target_data.right_spd=-_ctx.cmd->fric_target_speed;
    
    
    _ctx.data.out_data.l_fric_torque=_ctx.pid.l_fric_pid
                        ->calculate(_ctx.data.target_data.left_spd,
                                    _ctx.data.current_data.left_spd);
    _ctx.data.out_data.r_fric_torque=_ctx.pid.r_fric_pid
                        ->calculate(_ctx.data.target_data.right_spd,
                                    _ctx.data.current_data.right_spd);

    //在下力时先刹车而不是随惯性转
    if(fabs(_ctx.data.target_data.left_spd)<1e-5 && fabs(_ctx.data.current_data.left_spd)<1e-5)
        _ctx.data.out_data.l_fric_torque=0.0f;
    if(fabs(_ctx.data.target_data.right_spd)<1e-5 && fabs(_ctx.data.current_data.right_spd)<1e-5)
        _ctx.data.out_data.r_fric_torque=0.0f;                            



}

void sentry_booster_t::_trigger_control(){





if(_ctx.cmd->fric_on && _ctx.data.fric_ready==true){

    if(_ctx.cmd->multi_shoot == true){
        float distance =loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos - _ctx.data.current_data.trigger_equal_pos, -PI,PI) ;
        
         if(distance < PI/4){
        _ctx.data.target_data.trigger_equal_pos += PI/4; //连发时更新目标值，保证退出时单发在特定位置
        }
    }    
    else if(_ctx.cmd->fire_count  >  _ctx.data.current_fire_count){
        _ctx.data.target_data.trigger_equal_pos += PI/4;
        _ctx.data.current_fire_count++;
    }
    
}
    _ctx.data.target_data.trigger_equal_pos = loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos, -PI, PI);
    float error_trigger_pos    = 
                    loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos - _ctx.data.current_data.trigger_equal_pos,
                                         -PI, PI);

    float out_trigger_spd       = _ctx.pid.trig_pos_pid
                        ->calculate(error_trigger_pos,0.0f);

    if(_ctx.cmd->fric_on && _ctx.cmd->multi_shoot == true){

        out_trigger_spd = BOOSTER_SHOOT_TRIG_SPEED*BOOSTER_TRIGGER_RATE*BOOSTER_ONE_SHOOT_ANGLE;

    }
    _ctx.data.out_data.trigger_torque= _ctx.pid.trig_spd_pid
                        ->calculate(out_trigger_spd,
                                    _ctx.data.current_data.trigger_spd);
                                
        
}

void sentry_booster_t::_send_fric_command() const{ 
    _ctx.motor.fric_left->send_torque(_ctx.data.out_data.l_fric_torque);
    _ctx.motor.fric_right->send_torque(_ctx.data.out_data.r_fric_torque);

    }
void sentry_booster_t::_send_trig_command() const{
    _ctx.motor.trigger->send_torque(_ctx.data.out_data.trigger_torque);

}
void sentry_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

} // namespace pyro
