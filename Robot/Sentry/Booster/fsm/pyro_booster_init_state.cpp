#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_algo_common.h"

namespace pyro
{
static uint16_t trigger_init_count = 0;
void sentry_booster_t::fsm_active_t::state_init_t::enter(owner *owner)
{ 
    
    owner->_ctx.data._init_finish = false;
    trigger_init_count = 0;
}

void sentry_booster_t::fsm_active_t::state_init_t::execute(owner *owner)
{
    
    
       
        float trigger_init_speed = BOOSTER_TRIGGER_INIT_SPEED*BOOSTER_ONE_SHOOT_ANGLE*BOOSTER_TRIGGER_RATE;//以2颗/s 装弹
        owner->_ctx.data.out_data.trigger_torque =owner->_ctx.pid.trig_init_pid
                        ->calculate(trigger_init_speed,
                                    owner->_ctx.data.current_data.trigger_spd);   //接在sendtouque前

        if(owner->_ctx.data._init_finish ==0
                     && fabs(owner->_ctx.data.current_data.trigger_spd) <= BOOSTER_TRIGGER_PLUG_SPEED*BOOSTER_ONE_SHOOT_ANGLE*BOOSTER_TRIGGER_RATE
                    ){
                        trigger_init_count++;
            if(trigger_init_count >= BOOSTER_TRIGGER_INIT_COUNT){

                owner->_ctx.data.init_trigger_pos = owner->_ctx.data.current_data.trigger_equal_pos - 0.03f; //单发校准点位
                owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.init_trigger_pos;
                owner->_ctx.data._init_finish = 1 ; //初始化事件完成标志 
            }
        }
        else{
            trigger_init_count==0;
        }


        if( owner->_ctx.data._init_finish == 1){
            owner->_ctx.data.target_data.trigger_equal_pos = loop_fp32_constrain(owner->_ctx.data.target_data.trigger_equal_pos, -PI, PI);
            float error_trigger_pos    = 
                    loop_fp32_constrain(owner->_ctx.data.target_data.trigger_equal_pos - owner->_ctx.data.current_data.trigger_equal_pos,
                                         -PI, PI);

            float out_trigger_spd       = owner->_ctx.pid.trig_pos_pid
                        ->calculate(error_trigger_pos,0.0f);

 
            owner->_ctx.data.out_data.trigger_torque= owner->_ctx.pid.trig_spd_pid
                        ->calculate(out_trigger_spd,
                                    owner->_ctx.data.current_data.trigger_spd);
        }
        owner->_send_trig_command();
        owner->_ctx.motor.fric_left->send_torque(0);
        owner->_ctx.motor.fric_right->send_torque(0);
        if( owner->_ctx.data._init_finish == 1 && 
            fabs(owner->_ctx.data.target_data.trigger_equal_pos - owner->_ctx.data.current_data.trigger_equal_pos)<0.01f ){
                request_switch(&owner->_state_active._state_ready);
            }
        //不知道在不发单是要不要上力
        //owner->_ctx.motor.trigger->send_torque(0);

}

void sentry_booster_t::fsm_active_t::state_init_t::exit(owner *owner)
{
   // owner->_ctx.data._init_finish = 0 ;
}

} // namespace pyro


